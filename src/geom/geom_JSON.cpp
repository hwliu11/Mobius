//-----------------------------------------------------------------------------
// Created on: 18 June 2018
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
#include <mobius/geom_JSON.h>

//-----------------------------------------------------------------------------

//! Constructor accepting JSON string to process.
//! \param[in] json string representing JSON to process.
mobius::geom_JSON::geom_JSON(const std::string& json)
: core_JSON(json)
{}

//-----------------------------------------------------------------------------

//! Destructor.
mobius::geom_JSON::~geom_JSON()
{
}

//-----------------------------------------------------------------------------

bool mobius::geom_JSON::ExtractBCurve(core_Ptr<bcurve>& curve) const
{
  // Extract degree.
  int p = 0;
  if ( !this->ExtractNumericBlockForKey<int>("degree", p, 0) )
    return false;

  // Extract knot vector.
  std::vector<double> U;
  if ( !this->ExtractVector1d("knots", U) )
    return false;

  // Extract poles.
  std::vector<xyz> poles;
  if ( !this->ExtractVector3d("poles", poles) )
    return false;

  // Construct B-curve.
  curve = new bcurve(poles, U, p);
  return true;
}

//-----------------------------------------------------------------------------

bool mobius::geom_JSON::ExtractBSurface(core_Ptr<bsurf>& surface) const
{
  // Extract degrees.
  int p = 0, q = 0;
  if ( !this->ExtractNumericBlockForKey<int>("U_degree", p, 0) )
    return false;
  if ( !this->ExtractNumericBlockForKey<int>("V_degree", q, 0) )
    return false;

  // Extract knot vectors.
  std::vector<double> U, V;
  if ( !this->ExtractVector1d("U_knots", U) )
    return false;
  if ( !this->ExtractVector1d("V_knots", V) )
    return false;

  // Extract poles.
  std::vector< std::vector<xyz> > poles;
  if ( !this->ExtractGrid3d("poles", poles) )
    return false;

  // Costruct B-surface.
  surface = new bsurf(poles, U, V, p, q);
  return true;
}
