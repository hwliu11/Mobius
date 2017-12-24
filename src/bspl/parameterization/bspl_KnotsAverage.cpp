//-----------------------------------------------------------------------------
// Created on: 02 November 2013
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
#include <mobius/bspl_KnotsAverage.h>

//! Recognizes the averaging context.
//! \param D1_start [in] indicates whether D1 derivative constraint is requested
//!                      at the starting point of the curve being reconstructed.
//! \param D1_end   [in] indicates whether D1 derivative constraint is requested
//!                      at the trailing point of the curve being reconstructed.
//! \param D2_start [in] indicates whether D2 derivative constraint is requested
//!                      at the starting point of the curve being reconstructed.
//! \param D2_end   [in] indicates whether D2 derivative constraint is requested
//!                      at the trailing point of the curve being reconstructed.
//! \return averaging context.
mobius::bspl_KnotsAverage::Case
  mobius::bspl_KnotsAverage::Recognize(const bool D1_start,
                                       const bool D1_end,
                                       const bool D2_start,
                                       const bool D2_end)
{
  // TODO: currently averaging technique can process only a limited
  //       number of cases. It is not an easy question how to pre-select
  //       knots when many constraints are specified. Selection of knots
  //       has a great impact on the resulting interpolant's geometry, so
  //       one cannot simply choose "just any" knot vector and expect good
  //       geometry at the end

  if ( !D1_start && !D1_end && !D2_start && !D2_end )
    return Case_PointsOnly;

  if ( D1_start && D1_end && !D2_start && !D2_end )
    return Case_Points_D1;

  if ( D1_start && D1_end && D2_start && D2_end )
    return Case_Points_D1_D2;

  return Case_Unhandled;
}
