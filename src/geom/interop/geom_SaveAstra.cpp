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

bool geom_SaveAstra::Perform(const std::string&                  filename,
                             const std::vector<t_ptr<t_bcurve>>& curves,
                             const std::vector<t_ptr<t_bsurf>>&  surfaces,
                             const std::vector<t_surfOfRev>&     surfOfRev)
{
  setlocale(LC_CTYPE, ".1251");
  std::ofstream outfile;
  outfile.open(filename, std::ofstream::trunc);
  outfile.clear();
  outfile.close();
  outfile.open(filename, std::ios_base::app);
  //
  // Curves
  std::vector<std::pair<std::string, std::vector<std::vector<double>>>> resCurves;
  for (auto& curve : curves)
  {
    auto name = curve->GetName();
    auto knots = curve->GetKnots();
    std::vector<std::vector<double>> pnts;
    // clear knots from dublicates
    double prev = 0.;
    for (auto& knot : knots)
    {
      mobius::t_xyz p;
      curve->Eval(knot, p);
      auto t = curve->D1(knot);
      auto vec = std::vector<double>{p.X(), p.Y(), p.Z(), t.X(), t.Y(), t.Z(), knot};
      if (!pnts.empty() && abs(prev - knot) < 0.0001)
        continue;
      pnts.push_back(vec);
      prev = knot;
    }
    //
    outfile << name << " " << 7 << " " << pnts.size() << " 1 30 0 0\n";
    for (size_t i = 0; i < pnts.size(); ++i)
    {
      outfile << pnts[i][0] << " " << pnts[i][1] << " " << pnts[i][2] << " "
              << pnts[i][3] << " " << pnts[i][4] << " " << pnts[i][5] << " " << pnts[i][6] << "\n";
    }
    resCurves.push_back({name, pnts});
  }
  //
  // Surface of revolution
  for (auto& surf : surfOfRev)
  {
    outfile << surf.name << ' ' << "8 1 1 41 0 0\n";
    auto dir = surf.direction;
    auto loc = surf.location;
    outfile << "99**" << surf.curveName.substr(2) << ' ' << loc.X() << ' ' << loc.Y() << ' ' << loc.Z() << '\n'
                               << ' ' << dir.X() << ' ' << dir.Y() << ' ' << dir.Z() << '\n';
  }
  //
  // Surfaces
  std::vector<std::pair<std::string, std::vector<std::vector<double>>>> resSurfaces;
  for (auto& surface : surfaces)
  {
    auto name = surface->GetName();
    auto uKnots = surface->GetKnots_U();
    auto vKnots = surface->GetKnots_V();
    std::vector<std::vector<double>> pnts;
    // clear from duplicates
    double vPrev = 0.;
    int vKnotsNum = 0;
    int uKnotsNum = 0;
    for (auto& vKnot : vKnots)
    {
      if (!pnts.empty() && vPrev == vKnot)
        continue;
      double uPrev = 0.;
      uKnotsNum = 0;
      bool isFirst = true;
      for (auto& uKnot : uKnots)
      {
        t_xyz p, dU, dV, d2U, d2V, d2UV;
        surface->Eval_D2(uKnot, vKnot, p, dU, dV, d2U, d2V, d2UV);
        //
        if (!isFirst && abs(uPrev - uKnot) < 0.0001)
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
    outfile << name << ' ' << "14" << ' ' << uKnotsNum << ' ' << vKnotsNum << " 42 0 0\n";
    for (size_t i = 0; i < pnts.size(); ++i)
    {
      outfile << pnts[i][0] << " " << pnts[i][1] << " " << pnts[i][2] << " "
              << pnts[i][3] << " " << pnts[i][4] << " " << pnts[i][5] << " " << pnts[i][6] << " "
              << pnts[i][7] << " " << pnts[i][8] << " " << pnts[i][9] << " " << pnts[i][10] << " "
              << pnts[i][11] << " " << pnts[i][12] << " " << pnts[i][13] << "\n";
    }
    resSurfaces.push_back({name, pnts});
  }
  outfile.close();
  //
  return true;
}
