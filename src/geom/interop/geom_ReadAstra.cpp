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

// Standard includes
#include <fstream>

using namespace mobius;

//-----------------------------------------------------------------------------

#define Astra_ArrayHeaderSize 7
#define Astra_CurvePointSize  7
#define Astra_SurfPointSize   14

namespace {

  //-----------------------------------------------------------------------------

  //! Curve DTO.
  struct t_curveData
  {
    //! Curve point.
    struct t_pt
    {
      double x, y, z, tx, ty, tz, u;
      t_pt() : x(0.), y(0.), z(0.), tx(0.), ty(0.), tz(0.), u(0.) {}
    };

    std::string       name; //!< Curve name.
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
      pt.x  = core::str::to_number<double>(tokens[0]);
      pt.y  = core::str::to_number<double>(tokens[1]);
      pt.z  = core::str::to_number<double>(tokens[2]);
      pt.tx = core::str::to_number<double>(tokens[3]);
      pt.ty = core::str::to_number<double>(tokens[4]);
      pt.tz = core::str::to_number<double>(tokens[5]);
      pt.u  = core::str::to_number<double>(tokens[6]);
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
        t_xyz P0 = t_xyz(pL.x, pL.y, pL.z);
        t_xyz P3 = t_xyz(pR.x, pR.y, pR.z);
        t_xyz P1 = P0 + (1./3.)*(umax - umin)*t_xyz(pL.tx, pL.ty, pL.tz);
        t_xyz P2 = P3 - (1./3.)*(umax - umin)*t_xyz(pR.tx, pR.ty, pR.tz);

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

  //! Surface DTO.
  struct t_surfData
  {
    //! Surface point.
    struct t_pt
    {
      double x, y, z, Pu_x, Pu_y, Pu_z, Pv_x, Pv_y, Pv_z, Puv_x, Puv_y, Puv_z, u, v;

      //! Default ctor.
      t_pt() : x     (0.), y     (0.), z     (0.),
               Pu_x  (0.), Pu_y  (0.), Pu_z  (0.),
               Pv_x  (0.), Pv_y  (0.), Pv_z  (0.),
               Puv_x (0.), Puv_y (0.), Puv_z (0.),
               u     (0.), v     (0.)
      {}
    };

    //! Ctor.
    t_surfData() : ptSerial(0) {}

    //! Adds a surface point from the passed tokens.
    void AddPoint(const std::vector<std::string>& tokens)
    {
      this->ptSerial++;

      const int j = (this->ptSerial - 1) / this->nptsU;
      const int i = (this->ptSerial - 1) % this->nptsU;

      t_pt pt;
      pt.x     = core::str::to_number<double>(tokens[0]);
      pt.y     = core::str::to_number<double>(tokens[1]);
      pt.z     = core::str::to_number<double>(tokens[2]);
      pt.Pu_x  = core::str::to_number<double>(tokens[3]);
      pt.Pu_y  = core::str::to_number<double>(tokens[4]);
      pt.Pu_z  = core::str::to_number<double>(tokens[5]);
      pt.Pv_x  = core::str::to_number<double>(tokens[6]);
      pt.Pv_y  = core::str::to_number<double>(tokens[7]);
      pt.Pv_z  = core::str::to_number<double>(tokens[8]);
      pt.Puv_x = core::str::to_number<double>(tokens[9]);
      pt.Puv_y = core::str::to_number<double>(tokens[10]);
      pt.Puv_z = core::str::to_number<double>(tokens[11]);
      pt.u     = core::str::to_number<double>(tokens[12]);
      pt.v     = core::str::to_number<double>(tokens[13]);

      this->pts[i][j] = pt;
    }

     //! Prepares polynomial patches.
    void ToBezierPatches(std::vector< t_ptr<t_bsurf> >& tiles) const
    {
      /*const int numUisos = this->nptsU;
      const int numVisos = this->nptsV;

      for ( int j = 0; j < numVisos - 1; ++j )
      {
        for ( int i = 0; i < numUisos - 1; ++i )
        {
          const t_pt& P1 = this->pts[j + numUisos*i]
          const int serialIdx = Astra_SurfPointSize*(iPoint + (numUisos)*jPoint);
        }
      }*/
      // TODO: NYI
    }

    //! Converts surface data to a B-spline surface.
    t_ptr<t_bsurf> ToBSplineSurface() const
    {
      // TODO: NYI
      return nullptr;
    }

    std::string                      name;     //!< Surface name.
    int                              nptsU;    //!< Number of points in the U direction.
    int                              nptsV;    //!< Number of points in the V direction.
    std::vector< std::vector<t_pt> > pts;      //!< Surface points in a grid.
    int                              ptSerial; //!< Serial index of a point.
  };

  //! Checks if the passed line tokens represent a curve.
  bool IsCurve(const std::vector<std::string>& tokens,
               t_curveData&                    data)
  {
    if ( tokens.size() != Astra_ArrayHeaderSize )
      return false;

    if ( tokens[1] != core::str::to_string<int>(Astra_CurvePointSize) )
      return false;

    data = t_curveData();
    data.name = tokens[0];
    data.npts = core::str::extract_int(tokens[2]);
    return true;
  }

  //! Checks if the passed line tokens represent a surface.
  bool IsSurface(const std::vector<std::string>& tokens,
                 t_surfData&                     data)
  {
    if ( tokens.size() != Astra_ArrayHeaderSize )
      return false;

    // <name> _14_ 2 61 42 0 0
    if ( tokens[1] != core::str::to_string<int>(Astra_SurfPointSize) )
      return false;

    data = t_surfData();
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
    Mode_Surface
  };

  std::vector<t_curveData> curveDs;
  std::vector<t_surfData>  surfDs;
  //
  t_curveData* pCurrentCurve = nullptr;
  t_surfData*  pCurrentSurf  = nullptr;

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

    // Check if it's a curve or a surface.
    t_curveData curveData;
    t_surfData  surfData;
    //
    if ( ::IsCurve(tokens, curveData) )
    {
      m_progress.SendLogMessage(MobiusInfo(Normal) << "Next curve is '%1' with %2 points."
                                                   << curveData.name << curveData.npts);

      mode = Mode_Curve;
      curveDs.push_back(curveData);
      pCurrentCurve = &curveDs.back();
      continue;
    }
    //
    else if ( ::IsSurface(tokens, surfData) )
    {
      m_progress.SendLogMessage(MobiusInfo(Normal) << "Next surface is '%1' with %2 U points "
                                                      "and %3 V points."
                                                   << surfData.name
                                                   << surfData.nptsU
                                                   << surfData.nptsV);

      mode = Mode_Surface;
      surfDs.push_back(surfData);
      pCurrentSurf = &surfDs.back();
      continue;
    }

    if ( mode == Mode_Curve )
    {
      if ( pCurrentCurve->pts.size() < pCurrentCurve->npts )
        pCurrentCurve->AddPoint(tokens);
      else
        mode = Mode_Scan;
    }
    else if ( mode == Mode_Surface )
    {
      if ( pCurrentSurf->pts.size() < pCurrentSurf->nptsU*pCurrentSurf->nptsV )
        pCurrentSurf->AddPoint(tokens);
      else
        mode = Mode_Scan;
    }
  } // Until EOF.

  /* =========================================
   *  Stage 2: construct geometric primitives.
   * ========================================= */

  // Make curves.
  for ( auto& cds : curveDs )
    m_curves.push_back( cds.ToBSplineCurve() );

  // Make surface.
  for ( auto& sds : surfDs )
    m_surfaces.push_back( sds.ToBSplineSurface() );

  FILE.close();
  return true;
}
