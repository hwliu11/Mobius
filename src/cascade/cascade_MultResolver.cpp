//-----------------------------------------------------------------------------
// Created on: 24 December 2014
//-----------------------------------------------------------------------------
// Copyright (c) 2017, Sergey Slyadnev
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
#include <mobius/cascade_MultResolver.h>

//-----------------------------------------------------------------------------

//! Default constructor.
mobius::cascade_MultResolver::cascade_MultResolver() : bspl_MultResolver()
{}

//-----------------------------------------------------------------------------

//! \return knot array in OCCT form.
Handle(TColStd_HArray1OfReal)
  mobius::cascade_MultResolver::GetOpenCascadeKnots() const
{
  Handle(TColStd_HArray1OfReal)
    res = new TColStd_HArray1OfReal( 0, int( Knots.size() ) - 1 );
  //
  for ( int i = 0; i < (int)Knots.size(); ++i )
  {
    res->SetValue( i, Knots[i].u );
  }
  return res;
}

//-----------------------------------------------------------------------------

//! \return multiplicity array in OCCT form.
Handle(TColStd_HArray1OfInteger)
  mobius::cascade_MultResolver::GetOpenCascadeMults() const
{
  Handle(TColStd_HArray1OfInteger)
    res = new TColStd_HArray1OfInteger( 0, int( Knots.size() ) - 1 );
  //
  for ( int i = 0; i < (int)Knots.size(); ++i )
  {
    res->SetValue( i, Knots[i].m );
  }
  return res;
}
