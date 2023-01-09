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

#define Astra_CurveArraySize 7

//-----------------------------------------------------------------------------

namespace {

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

  //! Checks if the passed line tokens represent a curve.
  bool IsCurve(const std::vector<std::string>& tokens,
               t_curveData&                    data)
  {
    if ( tokens.size() != Astra_CurveArraySize )
      return false;

    if ( tokens[1] != core::str::to_string<int>(Astra_CurveArraySize) )
      return false;

    data = t_curveData();
    data.name = tokens[0];
    data.npts = core::str::extract_int(tokens[2]);
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

  enum Mode
  {
    Mode_Scan = 1,
    Mode_Curve
  };

  std::vector<t_curveData> curveDs;
  t_curveData* pCurrentCurve = nullptr;

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

    // Check if it's a curve.
    t_curveData curveData;
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

    if ( mode == Mode_Curve )
    {
      if ( pCurrentCurve->pts.size() < pCurrentCurve->npts )
        pCurrentCurve->AddPoint(tokens);
      else
        mode = Mode_Scan;
    }

    // TODO: NYI

  } // Until EOF.

  // Make curves.
  for ( auto& cds : curveDs )
    m_curves.push_back( cds.ToBSplineCurve() );

  FILE.close();
  return true;
}
