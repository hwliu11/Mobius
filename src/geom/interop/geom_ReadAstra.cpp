//-----------------------------------------------------------------------------
// Created on: 05 January 2023
//-----------------------------------------------------------------------------
// Copyright (c) 2013-present, Sergey Slyadnev
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//    * Neither the name of Sergey Slyadnev nor the
//      names of all contributors may be used to endorse or promote products
//      derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//-----------------------------------------------------------------------------

// Own include
#include <mobius/geom_ReadAstra.h>

// Core includes
#include <mobius/core_Axis.h>

// Geom includes
#include <mobius/geom_BSplineSurface.h>
#include <mobius/geom_SurfaceOfRevolution.h>

// Standard includes
#include <fstream>
#include <iterator>
#include <unordered_map>

using namespace mobius;

//-----------------------------------------------------------------------------

#define Astra_ArrayHeaderSize      7
#define Astra_SplineCurvePointSize 7
#define Astra_SplineSurfPointSize  14
#define Astra_RevolSurfPointSize   8
#define Astra_Ref                  "**"

namespace {

  //! Base class for all geometric entities.
  struct t_entity
  {
    std::string name; //!< Unicode name of the geometric entity.
  };

  //-----------------------------------------------------------------------------

  //! Curve DTO.
  struct t_curveData : public t_entity
  {
    //! Curve point.
    struct t_pt
    {
      t_xyz  P, Pt;
      double u;
      t_pt() : u(0.) {}
    };

    int               npts; //!< Number of points.
    std::vector<t_pt> pts;  //!< Curve points.

    //! Default ctor.
    t_curveData() : npts(0) {}

    //! Returns the number of segments.
    int GetNbSegments() const
    {
      return npts - 1;
    }

    //! Adds a curve point from the passed tokens.
    void AddPoint(const std::vector<std::string>& tokens)
    {
      t_pt pt;
      pt.P .SetX ( core::str::to_number<double>(tokens[0]) );
      pt.P .SetY ( core::str::to_number<double>(tokens[1]) );
      pt.P .SetZ ( core::str::to_number<double>(tokens[2]) );
      pt.Pt.SetX ( core::str::to_number<double>(tokens[3]) );
      pt.Pt.SetY ( core::str::to_number<double>(tokens[4]) );
      pt.Pt.SetZ ( core::str::to_number<double>(tokens[5]) );
      pt.u = core::str::to_number<double>(tokens[6]);
      //
      pts.push_back(pt);
    }

    //! Prepares polynomial segments.
    void ToBezierSegments(std::vector< t_ptr<t_bcurve> >& segments) const
    {
      const int nseg = this->GetNbSegments();
      //
      for ( int s = 0; s < nseg; ++s )
      {
        const t_pt&  pL   = this->pts[s];
        const t_pt&  pR   = this->pts[s + 1];
        const double umin = pL.u;
        const double umax = pR.u;

        // Compute control points of the corresponding Bezier segment.
        t_xyz P0 = pL.P;
        t_xyz P3 = pR.P;
        t_xyz P1 = P0 + (1./3.)*(umax - umin)*pL.Pt;
        t_xyz P2 = P3 - (1./3.)*(umax - umin)*pR.Pt;

        // Create Bezier segment.
        t_ptr<t_bcurve> seg = t_bcurve::MakeBezier(umin, umax, {P0, P1, P2, P3});
        //
        segments.push_back(seg);
      }
    }

    //! Converts curve data to a B-spline curve.
    t_ptr<t_bcurve> ToBSplineCurve() const
    {
      // Convert to Bezier segments.
      std::vector< t_ptr<t_bcurve> > segments;
      //
      this->ToBezierSegments(segments);

      if ( segments.empty() )
        return nullptr;

      // Concatenate curves.
      t_ptr<t_bcurve> res = segments[0];
      //
      for ( int i = 1; i < segments.size(); ++i )
        res->ConcatenateCompatible(segments[i]);

      return res;
    }
  };

  //-----------------------------------------------------------------------------

  //! Spline surface DTO.
  struct t_splineSurfData : public t_entity
  {
    //! Surface point.
    struct t_pt
    {
      t_xyz  P, Pu, Pv, Puv;
      double u, v;
      t_pt() : u(0.), v(0.) {}
    };

    //! Ctor.
    t_splineSurfData() : nptsU(0), nptsV(0), ptSerial(0) {}

    //! Adds a surface point from the passed tokens.
    void AddPoint(const std::vector<std::string>& tokens)
    {
      this->ptSerial++;

      const int j = (this->ptSerial - 1) / this->nptsU;
      const int i = (this->ptSerial - 1) % this->nptsU;

      t_pt pt;
      pt.P.SetX   ( core::str::to_number<double>(tokens[0]) );
      pt.P.SetY   ( core::str::to_number<double>(tokens[1]) );
      pt.P.SetZ   ( core::str::to_number<double>(tokens[2]) );
      pt.Pu.SetX  ( core::str::to_number<double>(tokens[3]) );
      pt.Pu.SetY  ( core::str::to_number<double>(tokens[4]) );
      pt.Pu.SetZ  ( core::str::to_number<double>(tokens[5]) );
      pt.Pv.SetX  ( core::str::to_number<double>(tokens[6]) );
      pt.Pv.SetY  ( core::str::to_number<double>(tokens[7]) );
      pt.Pv.SetZ  ( core::str::to_number<double>(tokens[8]) );
      pt.Puv.SetX ( core::str::to_number<double>(tokens[9]) );
      pt.Puv.SetY ( core::str::to_number<double>(tokens[10]) );
      pt.Puv.SetZ ( core::str::to_number<double>(tokens[11]) );
      pt.u = core::str::to_number<double>(tokens[12]);
      pt.v = core::str::to_number<double>(tokens[13]);

      this->pts[i][j] = pt;
    }

    //! Prepares polynomial patches.
    void ToBezierPatches(std::vector< std::vector< t_ptr<t_bsurf> > >& tiles) const
    {
      for ( int j = 0; j < this->nptsV - 1; ++j )
      {
        std::vector< t_ptr<t_bsurf> > row;

        for ( int i = 0; i < this->nptsU - 1; ++i )
        {
          const double umin = this->pts[i]  [j]  .u;
          const double umax = this->pts[i+1][j]  .u;
          const double vmin = this->pts[i]  [j]  .v;
          const double vmax = this->pts[i]  [j+1].v;

          t_xyz P00 = this->pts[i][j].P;

          t_xyz P10 = P00 + (1/3)*(umax-umin)*this->pts[i][j].Pu;
          t_xyz P01 = P00 + (1/3)*(vmax-vmin)*this->pts[i][j].Pv;
          t_xyz P11 = P10 + P01 - P00 + (1/9)*(umax-umin)*(vmax-vmin)*this->pts[i][j].Puv;

          t_xyz P30 = this->pts[i+1][j].P;
          t_xyz P20 = P30 - (1/3)*(umax-umin)*this->pts[i+1][j].Pu;
          t_xyz P31 = P30 + (1/3)*(vmax-vmin)*this->pts[i+1][j].Pv;
          t_xyz P21 = P20 + P31 - P30 - (1/9)*(umax-umin)*(vmax-vmin)*this->pts[i+1][j].Puv;

          t_xyz P03 = this->pts[i][j+1].P;
          t_xyz P13 = P03 + (1/3)*(umax-umin)*this->pts[i][j+1].Pu;
          t_xyz P02 = P03 - (1/3)*(vmax-vmin)*this->pts[i][j+1].Pv;
          t_xyz P12 = P13 + P02 - P03 - (1/9)*(umax-umin)*(vmax-vmin)*this->pts[i][j+1].Puv;

          t_xyz P33 = this->pts[i+1][j+1].P;
          t_xyz P23 = P33 - (1/3)*(umax-umin)*this->pts[i+1][j+1].Pu;
          t_xyz P32 = P33 - (1/3)*(vmax-vmin)*this->pts[i+1][j+1].Pv;
          t_xyz P22 = P23 + P32 - P33 + (1/9)*(umax-umin)*(vmax-vmin)*this->pts[i+1][j+1].Puv;

            /*{P00, P10, P20, P30},
              {P01, P11, P21, P31},
              {P02, P12, P22, P32},
              {P03, P13, P23, P33}*/

          std::vector< std::vector<t_xyz> >
            bzPoints = { {P00, P01, P02, P03},
                         {P10, P11, P12, P13},
                         {P20, P21, P22, P23},
                         {P30, P31, P32, P33} };

          t_ptr<t_bsurf> tile = t_bsurf::MakeBezier(umin, umax, vmin, vmax, bzPoints);
          //
          row.push_back(tile);
        }

        tiles.push_back(row);
      }
    }

    //! Converts surface data to a B-spline surface.
    t_ptr<t_bsurf> ToBSplineSurface()
    {
      // Convert to Bezier patches.
      this->ToBezierPatches(bzPatches);

      if ( bzPatches.empty() )
        return nullptr;

      const size_t nRows = bzPatches[0].size();
      const size_t nCols = bzPatches.size();

      // Concatenate patches along U direction.
      for ( int i = 0; i < nCols; ++i )
      {
        t_ptr<t_bsurf> res = bzPatches[i][0];
        //
        for ( size_t j = 1; j < nRows; ++j )
          res->ConcatenateCompatible(bzPatches[i][j], true);
        //
        splRows.push_back(res);
      }

      // Concatenate patches along V direction.
      t_ptr<t_bsurf> res = splRows[0];
      //
      for ( int i = 1; i < nCols; ++i )
      {
        res->ConcatenateCompatible(splRows[i], false);
      }

      return res;
    }

    int                                          nptsU;     //!< Number of points in the U direction.
    int                                          nptsV;     //!< Number of points in the V direction.
    std::vector< std::vector<t_pt> >             pts;       //!< Surface points in a grid.
    int                                          ptSerial;  //!< Serial index of a point.
    std::vector< std::vector< t_ptr<t_bsurf> > > bzPatches; //!< All paving Bezier patches.
    std::vector< t_ptr<t_bsurf> >                splRows;   //!< Spline rows after concatenation along U.
  };

  //-----------------------------------------------------------------------------

  //! Surface of revolution DTO.
  struct t_revolSurfData : public t_entity
  {
    t_axis      axis;      //!< Turning axis.
    std::string curveName; //!< Unicode name of the meridian curve.
  };

  //-----------------------------------------------------------------------------

  //! Checks if the passed line tokens represent a curve.
  bool IsCurve(const std::vector<std::string>& tokens,
               t_curveData&                    data)
  {
    if ( tokens.size() != Astra_ArrayHeaderSize )
      return false;

    if ( tokens[1] != core::str::to_string<int>(Astra_SplineCurvePointSize) )
      return false;

    data = t_curveData();
    data.name = tokens[0];
    data.npts = core::str::extract_int(tokens[2]);
    return true;
  }

  //-----------------------------------------------------------------------------

  //! Checks if the passed line tokens represent a spline surface.
  bool IsSplineSurface(const std::vector<std::string>& tokens,
                       t_splineSurfData&               data)
  {
    if ( tokens.size() != Astra_ArrayHeaderSize )
      return false;

    // <name> _14_ 2 61 42 0 0
    if ( tokens[1] != core::str::to_string<int>(Astra_SplineSurfPointSize) )
      return false;

    data = t_splineSurfData();
    data.name  = tokens[0];                         // _<name>_ 14  2   61  42 0 0
    data.nptsU = core::str::extract_int(tokens[2]); //  <name>  14 _2_  61  42 0 0
    data.nptsV = core::str::extract_int(tokens[3]); //  <name>  14  2  _61_ 42 0 0

    // Reserve space for the network of poles.
    for ( size_t row = 0; row < data.nptsU; ++row )
    {
      data.pts.push_back({});
      data.pts.back().resize(data.nptsV);
    }

    return true;
  }

  //-----------------------------------------------------------------------------

  //! Checks if the passed line tokens represent a surface of revolution.
  bool IsSurfaceOfRevolution(const std::vector<std::string>& tokens,
                             t_revolSurfData&                data)
  {
    if ( tokens.size() != Astra_ArrayHeaderSize )
      return false;

    // <name> _8_ 1 1 41 0 0
    if ( tokens[1] != core::str::to_string<int>(Astra_RevolSurfPointSize) )
      return false;

    data = t_revolSurfData();
    data.name = tokens[0]; // _<name>_ 8 1 1 41 0 0

    return true;
  }
}

//-----------------------------------------------------------------------------

bool geom_ReadAstra::Perform(const std::string& filename)
{
  std::ifstream FILE(filename);
  //
  if ( !FILE.is_open() )
  {
    m_progress.SendLogMessage(MobiusErr(Normal) << "Cannot open ASTRA file.");
    return false;
  }

  /* =========================================================
   *  Stage 1: read data from file, no conversion to geometry.
   * ========================================================= */

  enum Mode
  {
    Mode_Scan = 1,
    Mode_Curve,
    Mode_SplineSurface,
    Mode_SurfaceOfRevolution
  };

  std::vector<t_curveData>      curveDs;
  std::vector<t_splineSurfData> splSurfDs;
  std::vector<t_revolSurfData>  revSurfDs;
  //
  t_curveData*      pCurrentCurve   = nullptr;
  t_splineSurfData* pCurrentSplSurf = nullptr;
  t_revolSurfData*  pCurrentRevSurf = nullptr;

  // Loop over the file.
  Mode mode = Mode_Scan;
  //
  while ( !FILE.eof() )
  {
    char str[1024];
    FILE.getline(str, 1024);

    // Save tokens to vector.
    std::vector<std::string> tokens;
    std::istringstream iss(str);
    std::copy( std::istream_iterator<std::string>(iss),
               std::istream_iterator<std::string>(),
               std::back_inserter< std::vector<std::string> >(tokens) );

    if ( !tokens.size() )
      continue;

    /* ==============
     *  Read headers.
     * ============== */

    // Check if it's a curve or a surface.
    t_curveData      curveData;
    t_splineSurfData splSurfData;
    t_revolSurfData  revSurfData;
    //
    if ( ::IsCurve(tokens, curveData) )
    {
      m_progress.SendLogMessage(MobiusInfo(Normal) << "Next spline curve is '%1' with %2 points."
                                                   << curveData.name << curveData.npts);

      mode = Mode_Curve;
      curveDs.push_back(curveData);
      pCurrentCurve = &curveDs.back();
      continue;
    }
    //
    else if ( ::IsSplineSurface(tokens, splSurfData) )
    {
      m_progress.SendLogMessage(MobiusInfo(Normal) << "Next spline surface is '%1' with %2 U points "
                                                      "and %3 V points."
                                                   << splSurfData.name
                                                   << splSurfData.nptsU
                                                   << splSurfData.nptsV);

      mode = Mode_SplineSurface;
      splSurfDs.push_back(splSurfData);
      pCurrentSplSurf = &splSurfDs.back();
      continue;
    }
    //
    else if ( ::IsSurfaceOfRevolution(tokens, revSurfData) )
    {
      m_progress.SendLogMessage(MobiusInfo(Normal) << "Next surface of revolution is '%1'."
                                                   << revSurfData.name);

      mode = Mode_SurfaceOfRevolution;
      revSurfDs.push_back(revSurfData);
      pCurrentRevSurf = &revSurfDs.back();
      continue;
    }

    /* =====================
     *  Read geometric data.
     * ===================== */

    if ( mode == Mode_Curve )
    {
      if ( pCurrentCurve->pts.size() < pCurrentCurve->npts )
        pCurrentCurve->AddPoint(tokens);
      else
        mode = Mode_Scan;
    }
    //
    else if ( mode == Mode_SplineSurface )
    {
      if ( pCurrentSplSurf->pts.size() < pCurrentSplSurf->nptsU*pCurrentSplSurf->nptsV )
        pCurrentSplSurf->AddPoint(tokens);
      else
        mode = Mode_Scan;
    }
    //
    else if ( mode == Mode_SurfaceOfRevolution )
    {
      if ( pCurrentRevSurf->curveName.empty() )
      {
        std::vector<std::string> curveNameChunks;
        core::str::split(tokens[0], Astra_Ref, curveNameChunks);
        //
        if ( curveNameChunks.size() == 2 )
        {
          pCurrentRevSurf->curveName = curveNameChunks[1];
        }
        else
        {
          m_progress.SendLogMessage(MobiusErr(Normal) << "Unexpected format of surface of revolution: "
                                                         "99**<curveName> is expected as the first string.");
          return false;
        }

        m_progress.SendLogMessage(MobiusInfo(Normal) << "Meridian curve name is '%1'."
                                                     << pCurrentRevSurf->curveName);

        // The following chunks contain the coordinates of the origin point for the
        // turning axis.

        /*
           PST     8    1    1   41    0    0
       >>> 99**st  0.0000000E+00 0.0000000E+00 0.0000000E+00
                   0.0000000E+00 0.0000000E+00 0.1000000E+01
         */

        if ( tokens.size() != 4 )
        {
          m_progress.SendLogMessage(MobiusErr(Normal) << "Unexpected format of surface of revolution: "
                                                         "not enough strings representing the origin point.");
          return false;
        }

        const double Ox = std::stod(tokens[1]);
        const double Oy = std::stod(tokens[2]);
        const double Oz = std::stod(tokens[3]);
        //
        pCurrentRevSurf->axis.SetPosition( t_xyz(Ox, Oy, Oz) );
      }
      else // curve name is already set
      {
        if ( tokens.size() != 3 )
        {
          m_progress.SendLogMessage(MobiusErr(Normal) << "Unexpected format of surface of revolution: "
                                                         "not enough strings representing the axis direction.");
          return false;
        }

        const double Dx = std::stod(tokens[0]);
        const double Dy = std::stod(tokens[1]);
        const double Dz = std::stod(tokens[2]);
        //
        pCurrentRevSurf->axis.SetDirection( t_xyz(Dx, Dy, Dz) );

        // Done with the definition.
        mode = Mode_Scan;
      }
    }
  } // Until EOF.

  /* =========================================
   *  Stage 2: construct geometric primitives.
   * ========================================= */

  std::unordered_map<std::string, t_ptr<core_OBJECT>> ENTITIES;

  // Make curves.
  for ( auto& cds : curveDs )
  {
    m_curves.push_back( cds.ToBSplineCurve() );
    m_curves.back()->SetName(cds.name);

    // Register entity with the name.
    ENTITIES.insert({cds.name, m_curves.back()});
  }

  // Make spline surfaces.
  for ( auto& sds : splSurfDs )
  {
    t_ptr<t_bsurf> res = sds.ToBSplineSurface();
    res->SetName(sds.name);

    // Register entity with the name.
    ENTITIES.insert({sds.name, res});

    //for ( const auto& row : sds.bzPatches )
    //  for ( const auto& bz : row )
    //    m_surfaces.push_back(bz);

    //for ( const auto& row : sds.splRows )
    //  m_surfaces.push_back(row);

    m_surfaces.push_back(res);
  }

  // Make surfaces of revolution.
  for ( auto& rev : revSurfDs )
  {
    // Get reference to the meridian curve.
    const auto curveRef = ENTITIES.find(rev.curveName);
    //
    if ( curveRef == ENTITIES.end() )
    {
      m_progress.SendLogMessage(MobiusErr(Normal) << "The meridian curve '%1' does not seem to exist."
                                                  << rev.curveName);
      return false;
    }

    // Get the meridian curve.
    t_ptr<t_curve> C = t_ptr<t_curve>::DownCast(curveRef->second);
    //
    if ( C.IsNull() )
    {
      m_progress.SendLogMessage(MobiusErr(Normal) << "The meridian curve '%1' is null."
                                                  << rev.curveName);
      return false;
    }

    // Check the axis of revolution.
    if ( rev.axis.GetDirection().Modulus() < core_Precision::Resolution3D() )
    {
      m_progress.SendLogMessage(MobiusErr(Normal) << "The axis of revolution is degenerated for the surface '%1'."
                                                  << rev.name);
      return false;
    }

    // Build the surface of revolution.
    t_ptr<t_surfRevol> res = new t_surfRevol(C, rev.axis);
    res->SetName(rev.name);

    m_surfaces.push_back(res);

    // Register entity with the name.
    ENTITIES.insert({rev.name, res});
  }

  FILE.close();
  return true;
}
