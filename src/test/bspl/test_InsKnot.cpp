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
#include <mobius/bspl_FindSpan.h>

//-----------------------------------------------------------------------------

bool
  mobius::test_InsKnot::insertKnotCurve(const ptr<bcurve>&         curve,
                                        const double               u,
                                        const int                  r,
                                        const std::vector<double>& refKnots)
{
  /* ====================================================
   *  Prepare arguments for the knot insertion algorithm
   * ==================================================== */

  // Get properties of the input curve.
  const std::vector<double>&   UP = curve->GetKnots();
  const int                    p  = curve->GetDegree();
  const std::vector<core_XYZ>& Pw = curve->GetPoles();
  const int                    np = int( Pw.size() ) - 1;

  // Find span the knot in question falls into.
  bspl_FindSpan FindSpan(UP, p);
  const int k = FindSpan(u);

  // Resolve multiplicity
  int s = 0;
  for ( size_t i = 0; i < UP.size(); ++i )
    if ( UP[i] == u )
      s++;

  // Output arguments.
  int       nq = 0;
  const int mq = np + p + 1 + r;
  //
  std::vector<double> UQ; UQ.resize(mq + 1);
  std::vector<xyz> Qw;

  /* ==================================
   *  Perform knot insertion algorithm
   * ================================== */

  bspl_InsKnot InsKnot;

  if ( !InsKnot(np, p, UP, Pw, u, k, s, r, nq, UQ, Qw) )
    return false;

  /* ===================
   *  Verify the result
   * =================== */

  if ( UQ != refKnots )
    return false;

  return true;
}

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

  /* ==================================
   *  Perform knot insertion algorithm
   * ================================== */

  // Knot value to insert.
  const double u = 0.5;

  // How many times to insert.
  const int r = 1;

  // Reference data
  const std::vector<double> refUQ = {0, 0, 0, 0, 0.5, 1, 1, 1, 1};

  if ( !insertKnotCurve(curve, u, r, refUQ) )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

mobius::outcome mobius::test_InsKnot::test02(const int funcID)
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

  /* ==================================
   *  Perform knot insertion algorithm
   * ================================== */

  // Knot value to insert.
  const double u = 0.5;

  // How many times to insert.
  const int r = 2;

  // Reference data
  const std::vector<double> refUQ = {0, 0, 0, 0, 0.5, 0.5, 1, 1, 1, 1};

  if ( !insertKnotCurve(curve, u, r, refUQ) )
    return res.failure();

  return res.success();
}
