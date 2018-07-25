//-----------------------------------------------------------------------------
// Created on: 15 June 2018
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
#include <mobius/test_BSplineCurve.h>

// geom includes
#include <mobius/geom_BSplineCurve.h>

//-----------------------------------------------------------------------------

//! Test scenario 001: evaluate B-curve in its domain.
//! \param[in] funcID function ID.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_BSplineCurve::evalInDomain(const int funcID)
{
  outcome res;

  /* ======================
   *  Prepare input points
   * ====================== */

  // Control points.
  std::vector<xyz> Q = { xyz(0.0, 0.0, 0.0),
                         xyz(1.0, 0.0, 0.0),
                         xyz(2.0, 0.0, 0.0),
                         xyz(3.0, 0.0, 0.0),
                         xyz(4.0, 0.0, 0.0),
                         xyz(5.0, 0.0, 0.0),
                         xyz(6.0, 0.0, 0.0),
                         xyz(7.0, 0.0, 0.0) };

  // Knot vector.
  const std::vector<double> U = {0.0, 0.0, 0.0, 1, 2, 3, 4, 4, 5, 5, 5};

  // Degree.
  const int p = 2;

  // Construct B-curve.
  core_Ptr<bcurve> curve = new bcurve(Q, U, p);

  /* ==============
   *  Perform test
   * ============== */

  const double eps = 1e-7;
  const double u   = 0.5;
  //
  const xyz P_ref(0.875, 0, 0);

  // Evaluate.
  xyz P;
  curve->Eval(u, P);

  // Check.
  if ( (P - P_ref).Modulus() > eps )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 002: evaluate B-curve out of its domain on the right.
//! \param[in] funcID function ID.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_BSplineCurve::evalOutDomainRight(const int funcID)
{
  outcome res;

  /* ======================
   *  Prepare input points
   * ====================== */

  // Control points.
  std::vector<xyz> Q = { xyz(0.0, 0.0, 0.0),
                         xyz(1.0, 0.0, 0.0),
                         xyz(2.0, 0.0, 0.0),
                         xyz(3.0, 0.0, 0.0),
                         xyz(4.0, 0.0, 0.0),
                         xyz(5.0, 0.0, 0.0),
                         xyz(6.0, 0.0, 0.0),
                         xyz(7.0, 0.0, 0.0) };

  // Knot vector.
  const std::vector<double> U = {0.0, 0.0, 0.0, 1, 2, 3, 4, 4, 5, 5, 5};

  // Degree.
  const int p = 2;

  // Construct B-curve.
  core_Ptr<bcurve> curve = new bcurve(Q, U, p);

  /* ==============
   *  Perform test
   * ============== */

  const double eps = 1e-7;
  const double u   = 5.5;
  //
  const xyz P_ref(8.0, 0, 0);

  // Evaluate.
  xyz P;
  curve->Eval(u, P);

  // Check.
  if ( (P - P_ref).Modulus() > eps )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 003: evaluate B-curve out of its domain on the left.
//! \param[in] funcID function ID.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_BSplineCurve::evalOutDomainLeft(const int funcID)
{
  outcome res;

  /* ======================
   *  Prepare input points
   * ====================== */

  // Control points.
  std::vector<xyz> Q = { xyz(0.0, 0.0, 0.0),
                         xyz(1.0, 0.0, 0.0),
                         xyz(2.0, 0.0, 0.0),
                         xyz(3.0, 0.0, 0.0),
                         xyz(4.0, 0.0, 0.0),
                         xyz(5.0, 0.0, 0.0),
                         xyz(6.0, 0.0, 0.0),
                         xyz(7.0, 0.0, 0.0) };

  // Knot vector.
  const std::vector<double> U = {0.0, 0.0, 0.0, 1, 2, 3, 4, 4, 5, 5, 5};

  // Degree.
  const int p = 2;

  // Construct B-curve.
  core_Ptr<bcurve> curve = new bcurve(Q, U, p);

  /* ==============
   *  Perform test
   * ============== */

  const double eps = 1e-7;
  const double u   = -1.0;
  //
  const xyz P_ref(-2.5, 0, 0);

  // Evaluate.
  xyz P;
  curve->Eval(u, P);

  // Check.
  if ( (P - P_ref).Modulus() > eps )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 004: evaluate B-curve defined as JSON object. The curve is
//! evaluated within its domain.
//! \param[in] funcID function ID.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_BSplineCurve::evalJSON1(const int funcID)
{
  outcome res;

  /* ======================
   *  Prepare input points
   * ====================== */

  // JSON definition.
  std::string json =
  "{\
    entity: curve,\
    type: b-curve,\
    continuity: C0,\
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
        degree: 1,\
        knots: [0, 0, 0.5, 1, 1],\
        num_poles: 3,\
        poles: [[10, -10, 0], [10, 0, 0], [10, 10, 0]]\
    }\
  }";

  // Construct B-curve.
  core_Ptr<bcurve> curve = bcurve::Instance(json);
  //
  if ( curve.IsNull() )
    return res.failure();

  /* ==============
   *  Perform test
   * ============== */

  const double eps = 1e-7;
  const double u   = 0.5;
  //
  const xyz P_ref(10.0, 0, 0);

  // Evaluate.
  xyz P;
  curve->Eval(u, P);

  // Check.
  if ( (P - P_ref).Modulus() > eps )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 005: evaluate B-curve defined as JSON object. The curve is
//! evaluated within its domain.
//! \param[in] funcID function ID.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_BSplineCurve::evalJSON2(const int funcID)
{
  outcome res;

  /* ======================
   *  Prepare input points
   * ====================== */

  // JSON definition.
  std::string json =
  "{\
    entity: curve,\
    type: b-curve,\
    continuity: C2,\
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
        knots: [0, 0, 0, 0, 0.086552659882147165, 0.12991610337519779, 0.17335320155205217, 0.21685563217354173, 0.26042585254349176, 0.30407265714329168, 0.34777085278678926, 0.39149953433338203, 0.43523418673002129, 0.47897080289576599, 0.52270370522331799, 0.56640842709462558, 0.61006420615718915, 0.65364724150077347, 0.6971411482848936, 0.74053183264471611, 0.78381534587472068, 0.82699997530461111, 0.87012510506088203, 0.91327212141610736, 1, 1, 1, 1],\
        num_poles: 24,\
        poles: [[-47.245171837360445, -13.330112189819705, -7.6956009860000014], [-48.543230124118843, -13.306669608460778, -9.0548748651367035], [-50.459513200569567, -13.369730780838193, -11.105462588842185], [-53.120158946442487, -12.996104686238775, -13.823850023214577], [-55.150254257225164, -12.568772592486884, -15.862474062919576], [-57.203359871504929, -11.986234017726066, -17.895288576625656], [-59.270272317910752, -11.31256769219627, -19.914758239378756], [-61.353784624428712, -10.384652377421357, -21.954429387420479], [-63.412427061251812, -9.8681941325851206, -23.887130582565415], [-65.484887191100427, -8.6415701682777151, -25.946336002597189], [-67.515871836644521, -7.9214371451977916, -27.932877743490458], [-69.579986873492558, -6.6484848190495196, -29.929327424093149], [-71.587490412051366, -5.7394937592878108, -31.921845681675919], [-73.587957963254155, -4.8105535030404427, -33.912492469390827], [-75.587001806797687, -3.5650858465993895, -35.901074175424633], [-77.550199361145332, -2.7154826020692231, -37.890802604771608], [-79.506697712655722, -1.9614739408569175, -39.881664289945746], [-81.455999191167351, -0.95767986837677421, -41.871399498463624], [-83.393251263788315, -0.60519666694352103, -43.865764450167248], [-85.334615926858959, 0.032062381828010633, -45.85932051207741], [-87.273965252953516, -0.12444161078877045, -47.859727806027777], [-89.874234912535812, -0.11430251061433312, -50.526657644448242], [-91.812683725642572, -1.3724906661027914, -52.533684302239614], [-93.087872209687532, -1.8918153723992683, -53.83992385900001]]\
    }\
  }";

  // Construct B-curve.
  core_Ptr<bcurve> curve = bcurve::Instance(json);
  //
  if ( curve.IsNull() )
    return res.failure();

  /* ==============
   *  Perform test
   * ============== */

  const double eps = 1e-4;
  const double u   = 0.5;
  //
  const xyz P_ref(-70.5441, -6.21939, -30.8876);

  // Evaluate.
  xyz P;
  curve->Eval(u, P);

  // Check.
  if ( (P - P_ref).Modulus() > eps )
    return res.failure();

  return res.success();
}
