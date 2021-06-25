//-----------------------------------------------------------------------------
// Created on: 25 June 2021
//-----------------------------------------------------------------------------
// Copyright (c) 2021-present, Sergey Slyadnev
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
#include <mobius/core_BBox.h>

using namespace mobius;

//-----------------------------------------------------------------------------

void core_BBox::Get(double& xmin, double& ymin, double& zmin,
                    double& xmax, double& ymax, double& zmax) const
{
  xmin = minPt.X();
  ymin = minPt.Y();
  zmin = minPt.Z();
  xmax = maxPt.X();
  ymax = maxPt.Y();
  zmax = maxPt.Z();
}

//-----------------------------------------------------------------------------

void core_BBox::Add(const core_XYZ& P)
{
  if ( IsVoid )
  {
    minPt  = P;
    maxPt  = P;
    IsVoid = false;
  }
  else
  {
    if ( P.X() < minPt.X() ) minPt.SetX( P.X() );
    if ( P.Y() < minPt.Y() ) minPt.SetY( P.Y() );
    if ( P.Z() < minPt.Z() ) minPt.SetZ( P.Z() );
    if ( P.X() > maxPt.X() ) maxPt.SetX( P.X() );
    if ( P.Y() > maxPt.Y() ) maxPt.SetY( P.Y() );
    if ( P.Z() > maxPt.Z() ) maxPt.SetZ( P.Z() );
  }
}

//-----------------------------------------------------------------------------

void core_BBox::Add(const double x, const double y, const double z)
{
  this->Add( core_XYZ(x, y, z) );
}

//-----------------------------------------------------------------------------

bool core_BBox::IsOut(const core_XYZ& P,
                      const double    tolerance) const
{
  double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;
  this->Get(Xmin, Ymin, Zmin, Xmax, Ymax, Zmax);

  if ( P.X() < Xmin && std::fabs(P.X() - Xmin) > tolerance ) return true;
  if ( P.X() > Xmax && std::fabs(P.X() - Xmax) > tolerance ) return true;

  if ( P.Y() < Ymin && std::fabs(P.Y() - Ymin) > tolerance ) return true;
  if ( P.Y() > Ymax && std::fabs(P.Y() - Ymax) > tolerance ) return true;

  if ( P.Z() < Zmin && std::fabs(P.Z() - Zmin) > tolerance ) return true;
  if ( P.Z() > Zmax && std::fabs(P.Z() - Zmax) > tolerance ) return true;

  return false;
}

//-----------------------------------------------------------------------------

bool core_BBox::IsOut(const core_BBox& other,
                      const double     tolerance) const
{
  double OXmin, OYmin, OZmin, OXmax, OYmax, OZmax;
  other.Get(OXmin, OYmin, OZmin, OXmax, OYmax, OZmax);

  double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;
  this->Get(Xmin, Ymin, Zmin, Xmax, Ymax, Zmax);

  if ( OXmax < Xmin && std::fabs(OXmax - Xmin) > tolerance ) return true;
  if ( OXmin > Xmax && std::fabs(OXmin - Xmax) > tolerance ) return true;

  if ( OYmax < Ymin && std::fabs(OYmax - Ymin) > tolerance ) return true;
  if ( OYmin > Ymax && std::fabs(OYmin - Ymax) > tolerance ) return true;

  if ( OZmax < Zmin && std::fabs(OZmax - Zmin) > tolerance ) return true;
  if ( OZmin > Zmax && std::fabs(OZmin - Zmax) > tolerance ) return true;

  return false;
}
