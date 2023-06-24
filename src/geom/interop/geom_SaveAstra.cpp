//-----------------------------------------------------------------------------
// Created on: 05 June 2023
//-----------------------------------------------------------------------------
// Copyright (c) 2013-present, Elizaveta Krylova
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
//    * Neither the name of Elizaveta Krylova nor the
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
#include <mobius/geom_SaveAstra.h>

// Core includes
#include <mobius/core_Axis.h>

// Geom includes
#include <mobius/geom_SurfaceOfRevolution.h>

// Standard includes
#include <fstream>
#include <iterator>
#include <unordered_map>

using namespace mobius;

//-----------------------------------------------------------------------------

bool geom_SaveAstra::Perform(const std::string&                       filename,
                             const std::vector< t_ptr<t_bcurve> >&    bCurves,
                             const std::vector< t_ptr<t_bsurf> >&     bSurfaces,
                             const std::vector< t_ptr<t_surfRevol> >& revolSurfaces)
{
#if defined WIN32
  setlocale(LC_CTYPE, ".1251");
#endif

  std::ofstream outfile(filename);
  //
  if ( !outfile.is_open() )
    return false;

  //outfile << std::setw(13) << std::setprecision(7) << std::scientific;

  // Make sure that generatrix curves of all surfaces of revolution are also stored.
  std::vector< t_ptr<t_bcurve> > allCurves = bCurves;
  //
  for ( const auto& revolSurf : revolSurfaces )
  {
    t_ptr<t_bcurve>
      meridian = t_ptr<t_bcurve>::DownCast( revolSurf->GetMeridian() );

    if ( meridian->GetName().empty() && !revolSurf->GetName().empty() )
      meridian->SetName( revolSurf->GetName() + "c" );

    allCurves.push_back(meridian);
  }

  // B-spline curves.
  std::vector<std::pair<std::string, std::vector<std::vector<double>>>> resCurves;
  //
  for ( auto& curve : allCurves )
  {
    const std::string&         name  = curve->GetName();
    const std::vector<double>& knots = curve->GetKnots();

    std::vector<std::vector<double>> pnts;

    // Clear knots from duplicates.
    double prev = 0.;
    for ( const double knot : knots )
    {
      t_xyz pt;
      curve->Eval(knot, pt);
      //
      t_xyz dC = curve->D1(knot);
      //
      if ( !pnts.empty() && abs(prev - knot) < core_Precision::Resolution3D() )
        continue;

      // Data to serialize.
      std::vector<double> vec{ pt.X(), pt.Y(), pt.Z(), dC.X(), dC.Y(), dC.Z(), knot };
      pnts.push_back(vec);

      prev = knot;
    }
    //
    outfile << core::str::fixedlen<std::string>(name,        4, true)
            << core::str::fixedlen<int>        (7,           5, false)
            << core::str::fixedlen<size_t>     (pnts.size(), 5, false)
            << core::str::fixedlen<int>        (1,           5, false)
            << core::str::fixedlen<int>        (30,          5, false)
            << core::str::fixedlen<int>        (0,           5, false)
            << core::str::fixedlen<int>        (0,           5, false)
            << "\n";
    //
    for ( size_t i = 0; i < pnts.size(); ++i )
    {
      for ( size_t k = 0; k < 7; ++k )
      {
        char buff[FORTRAN_BUFSIZE];
        outfile << core::str::fortranize(pnts[i][k], buff);
        //
        if ( k != 6 )
          outfile << " ";
      }
      outfile << "\n";
    }
    resCurves.push_back({name, pnts});
  }

  // Surfaces of revolution.
  for ( auto& surf : revolSurfaces )
  {
    const std::string& name      = surf->GetName();
    const std::string& curveName = surf->GetMeridian()->GetName();
    const t_axis&      ax        = surf->GetAxis();
    const t_xyz&       dir       = ax.GetDirection();
    const t_xyz&       loc       = ax.GetPosition();

    outfile << core::str::fixedlen<std::string>(name,      4, true)
            << core::str::fixedlen<int>        (8,         5, false)
            << core::str::fixedlen<int>        (1,         5, false)
            << core::str::fixedlen<int>        (1,         5, false)
            << core::str::fixedlen<int>        (41,        5, false)
            << core::str::fixedlen<int>        (0,         5, false)
            << core::str::fixedlen<int>        (0,         5, false)
            << "\n";

    outfile << "99**"
            << core::str::fixedlen<std::string>(curveName, 4, true)
            << "   ";

    // location X
    {
      char buff[FORTRAN_BUFSIZE];
      outfile << core::str::fortranize(loc.X(), buff) << " ";
    }

    // location Y
    {
      char buff[FORTRAN_BUFSIZE];
      outfile << core::str::fortranize(loc.Y(), buff) << " ";
    }

    // location Z
    {
      char buff[FORTRAN_BUFSIZE];
      outfile << core::str::fortranize(loc.Z(), buff) << " ";
    }

    // newline
    outfile << "\n           ";

    // direction X
    {
      char buff[FORTRAN_BUFSIZE];
      outfile << core::str::fortranize(dir.X(), buff) << " ";
    }

    // direction Y
    {
      char buff[FORTRAN_BUFSIZE];
      outfile << core::str::fortranize(dir.Y(), buff)<< " ";
    }

    // direction Z
    {
      char buff[FORTRAN_BUFSIZE];
      outfile << core::str::fortranize(dir.Z(), buff);
      outfile << "\n";
    }
  }

  // B-spline surfaces
  std::vector<std::pair<std::string, std::vector<std::vector<double>>>> resSurfaces;
  for (auto& surf : bSurfaces)
  {
    const std::string&         name   = surf->GetName();
    const std::vector<double>& uKnots = surf->GetKnots_U();
    const std::vector<double>& vKnots = surf->GetKnots_V();

    std::vector<std::vector<double>> pnts;
    // clear from duplicates
    double vPrev = 0.;
    int vKnotsNum = 0;
    int uKnotsNum = 0;
    for ( const double vKnot : vKnots )
    {
      if (!pnts.empty() && abs(vPrev - vKnot) < core_Precision::Resolution3D() )
        continue;
      double uPrev = 0.;
      uKnotsNum = 0;
      bool isFirst = true;
      for ( const double uKnot : uKnots )
      {
        t_xyz p, dU, dV, d2U, d2V, d2UV;
        surf->Eval_D2(uKnot, vKnot, p, dU, dV, d2U, d2V, d2UV);
        //
        if ( !isFirst && abs(uPrev - uKnot) < core_Precision::Resolution3D() )
          continue;

        isFirst = false;
        pnts.push_back({p.X(), p.Y(), p.Z(), dU.X(), dU.Y(), dU.Z(), dV.X(), dV.Y(), dV.Z(), d2UV.X(), d2UV.Y(), d2UV.Z(), uKnot, vKnot});
        uPrev = uKnot;
        uKnotsNum++;
      }
      vKnotsNum++;
      vPrev = vKnot;
    }
    //
    outfile << core::str::fixedlen<std::string>(name,      4, true)
            << core::str::fixedlen<int>        (14,        5, false)
            << core::str::fixedlen<int>        (uKnotsNum, 5, false)
            << core::str::fixedlen<int>        (vKnotsNum, 5, false)
            << core::str::fixedlen<int>        (42,        5, false)
            << core::str::fixedlen<int>        (0,         5, false)
            << core::str::fixedlen<int>        (0,         5, false)
            << "\n";
    //
    for (size_t i = 0; i < pnts.size(); ++i)
    {
      for ( size_t k = 0; k < 14; ++k )
      {
        char buff[FORTRAN_BUFSIZE];
        outfile << core::str::fortranize(pnts[i][k], buff);
        //
        if ( k != 13 )
          outfile << " ";
      }
      outfile << "\n";
    }
    resSurfaces.push_back({name, pnts});
  }
  outfile.close();
  //
  return true;
}
