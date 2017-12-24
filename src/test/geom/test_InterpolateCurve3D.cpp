//-----------------------------------------------------------------------------
// Created on: 04 November 2013
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
#include <mobius/test_InterpolateCurve3D.h>

// geom includes
#include <mobius/geom_InterpolateCurve.h>

//! Test scenario 001.
//! \param funcID [in] function ID.
//! \return true in case of success, false -- otherwise.
bool mobius::test_InterpolateCurve3D::test1(const int test_NotUsed(funcID))
{
  /* ~~~~~~~~~~~~~~~~~~~~~~
   *  Prepare input points
   * ~~~~~~~~~~~~~~~~~~~~~~ */

  xyz Q[5] = { xyz( 0.0,  0.0, 0.0),
               xyz( 3.0,  4.0, 0.0),
               xyz(-1.0,  4.0, 0.0),
               xyz(-4.0,  0.0, 0.0),
               xyz(-4.0, -3.0, 0.0) };

  std::vector<xyz> Q_vec;
  for ( int k = 0; k < sizeof(Q)/sizeof(xyz); ++k )
    Q_vec.push_back(Q[k]);

  /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   *  Run interpolation algorithm
   * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

  // Construct interpolation tool
  geom_InterpolateCurve Interp(Q_vec, 3, ParamsSelection_ChordLength, KnotsSelection_Average);

  // Run interpolation
  Interp.Perform();

  // Verify
  if ( Interp.ErrorCode() != geom_InterpolateCurve::ErrCode_NoError )
    return false;

  return true;
}
