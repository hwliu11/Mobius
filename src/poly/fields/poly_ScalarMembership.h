//-----------------------------------------------------------------------------
// Created on: 25 March 2020
//-----------------------------------------------------------------------------
// Copyright (c) 2020-present, Sergey Slyadnev
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

#ifndef poly_ScalarMembership_HeaderFile
#define poly_ScalarMembership_HeaderFile

// Poly includes
#include <mobius/poly.h>

namespace mobius {

//! \ingroup MOBIUS_POLY
//!
//! Membership classifiers for scalar fields.
enum poly_ScalarMembership
{
  ScalarMembership_None    = 0x00,
  //
  ScalarMembership_In      = 0x01,
  ScalarMembership_On      = 0x02,
  ScalarMembership_Out     = 0x04,
  //
  ScalarMembership_OnIn    = ScalarMembership_On | ScalarMembership_In,
  ScalarMembership_OnOut   = ScalarMembership_On | ScalarMembership_Out,
  ScalarMembership_OnInOut = ScalarMembership_On | ScalarMembership_In | ScalarMembership_Out
};

}

#endif
