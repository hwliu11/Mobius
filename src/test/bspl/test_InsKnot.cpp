//-----------------------------------------------------------------------------
// Created on: 07 December 2018
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
#include <mobius/test_InsKnot.h>

// Geom includes
#include <mobius/geom_BSplineCurve.h>

// bspl includes
#include <mobius/bspl_InsKnot.h>

//-----------------------------------------------------------------------------

mobius::outcome mobius::test_InsKnot::test01(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  /* =====================
   *  Prepare input curve
   * ===================== */

  // JSON definition.
  std::string json =
  "{\
    entity: curve,\
    type: b-curve,\
    continuity: CN,\
    domain: {\
        U_min: 0,\
        U_max: 1\
    },\
    flags: {\
        is_rational: 0,\
        is_periodic: 0,\
        is_closed: 0\
    },\
    properties: {\
        degree: 3,\
        knots: [0, 0, 0, 0, 1, 1, 1, 1],\
        num_poles: 4,\
        poles: [[-1.7273331185814795e-016, -2.7903073454008515e-016, -1.9930766752863225e-016], [2.1356662204292287, 0.61293215795463996, 0.86133743282055253], [-0.10849860924114609, 2.748069190766909, 2.0868682935855745], [2, 3, 4]]\
    }\
  }";

  // Construct B-curve.
  ptr<bcurve> curve = bcurve::Instance(json);
  //
  if ( curve.IsNull() )
    return res.failure();

  //// Prepare arguments for the knot insertion algorithm.
  //const std::vector<double> UP = curve->Knots;
  //const int                 np = int( UP.size() ) - 1;
  //const int                 p  = 1;
  //
  //             const std::vector<double>&   UP,
  //             const std::vector<core_XYZ>& Pw,
  //             const double                 u,
  //             const int                    k,
  //             const int                    s,
  //             const int                    r,
  //             int&                         nq,
  //             std::vector<double>&         UQ,
  //             std::vector<core_XYZ>&       Qw

  //bspl_InsKnot InsKnot(U, p);
  //int I1  = FindSpan(u1);
  //int I2  = FindSpan(u2);
  //int I3  = FindSpan(u3);
  //int I4  = FindSpan(u4);
  //int I5  = FindSpan(u5);
  //int I6  = FindSpan(u6);
  //int I7  = FindSpan(u7);
  //int I8  = FindSpan(u8);
  //int I9  = FindSpan(u9);
  //int I10 = FindSpan(u10);
  //int I11 = FindSpan(u11);
  //int I12 = FindSpan(u12);

  //if ( I1 == 1 && I2 == 1 && I3 == 2 && I4 == 2 && I5 == 2 && I6 == 3 &&
  //     I7 == 3 && I8 == 4 && I9 == 4 && I10 == 5 && I11 == 5 && I12 == 5 )
  //  return res.success();

  return res.failure();
}
