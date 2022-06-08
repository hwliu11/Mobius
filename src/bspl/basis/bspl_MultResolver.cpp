//-----------------------------------------------------------------------------
// Created on: 13 December 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2018-present, Sergey Slyadnev
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
#include <mobius/bspl_MultResolver.h>

//-----------------------------------------------------------------------------

mobius::bspl_MultResolver::bspl_MultResolver()
{}

//-----------------------------------------------------------------------------

mobius::bspl_MultResolver::bspl_MultResolver(const std::vector<double>& U)
{
  this->Resolve(U);
}

//-----------------------------------------------------------------------------

void mobius::bspl_MultResolver::Resolve(const double u)
{
  bool isFound = false;
  int foundIdx = -1;
  t_knot_multiset::elem foundStruct;
  //
  for ( int i = 0; i < (int)Knots.size(); ++i )
  {
    const t_knot_multiset::elem& knotWithMult = Knots[i];
    if ( fabs(knotWithMult.u - u) < DBL_EPSILON )
    {
      isFound     = true;
      foundIdx    = i;
      foundStruct = knotWithMult;
      break;
    }
  }
  if ( isFound )
  {
    foundStruct.m += 1;
    Knots[foundIdx] = foundStruct;
  }
  else
  {
    foundStruct.u = u;
    foundStruct.m = 1;
    Knots.push_back(foundStruct);
  }
}

//-----------------------------------------------------------------------------

void mobius::bspl_MultResolver::Resolve(const std::vector<double>& U)
{
  for ( size_t ii = 0; ii < U.size(); ++ii )
    this->Resolve(U[ii]);
}
