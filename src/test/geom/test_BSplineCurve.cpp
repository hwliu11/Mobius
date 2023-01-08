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

// bspl includes
#include <mobius/bspl.h>

// geom includes
#include <mobius/geom_BSplineCurve.h>

//-----------------------------------------------------------------------------

//! Test scenario 001: evaluate B-curve in its domain.
//! \param[in] funcID function ID.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_BSplineCurve::evalInDomain(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  /* ======================
   *  Prepare input curve
   * ====================== */

  // Control points.
  std::vector<t_xyz> Q = { t_xyz(0.0, 0.0, 0.0),
                           t_xyz(1.0, 0.0, 0.0),
                           t_xyz(2.0, 0.0, 0.0),
                           t_xyz(3.0, 0.0, 0.0),
                           t_xyz(4.0, 0.0, 0.0),
                           t_xyz(5.0, 0.0, 0.0),
                           t_xyz(6.0, 0.0, 0.0),
                           t_xyz(7.0, 0.0, 0.0) };

  // Knot vector.
  const std::vector<double> U = {0.0, 0.0, 0.0, 1, 2, 3, 4, 4, 5, 5, 5};

  // Degree.
  const int p = 2;

  // Construct B-curve.
  core_Ptr<t_bcurve> curve = new t_bcurve(Q, U, p);

  /* ==============
   *  Perform test
   * ============== */

  const double eps = 1e-7;
  const double u   = 0.5;
  //
  const t_xyz P_ref(0.875, 0, 0);

  // Evaluate.
  t_xyz P;
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
  outcome res( DescriptionFn(), funcID );

  /* ======================
   *  Prepare input curve
   * ====================== */

  // Control points.
  std::vector<t_xyz> Q = { t_xyz(0.0, 0.0, 0.0),
                           t_xyz(1.0, 0.0, 0.0),
                           t_xyz(2.0, 0.0, 0.0),
                           t_xyz(3.0, 0.0, 0.0),
                           t_xyz(4.0, 0.0, 0.0),
                           t_xyz(5.0, 0.0, 0.0),
                           t_xyz(6.0, 0.0, 0.0),
                           t_xyz(7.0, 0.0, 0.0) };

  // Knot vector.
  const std::vector<double> U = {0.0, 0.0, 0.0, 1, 2, 3, 4, 4, 5, 5, 5};

  // Degree.
  const int p = 2;

  // Construct B-curve.
  core_Ptr<t_bcurve> curve = new t_bcurve(Q, U, p);

  /* ==============
   *  Perform test
   * ============== */

  const double eps = 1e-7;
  const double u   = 5.5;
  //
  const t_xyz P_ref(8.0, 0, 0);

  // Evaluate.
  t_xyz P;
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
  outcome res( DescriptionFn(), funcID );

  /* ======================
   *  Prepare input curve
   * ====================== */

  // Control points.
  std::vector<t_xyz> Q = { t_xyz(0.0, 0.0, 0.0),
                           t_xyz(1.0, 0.0, 0.0),
                           t_xyz(2.0, 0.0, 0.0),
                           t_xyz(3.0, 0.0, 0.0),
                           t_xyz(4.0, 0.0, 0.0),
                           t_xyz(5.0, 0.0, 0.0),
                           t_xyz(6.0, 0.0, 0.0),
                           t_xyz(7.0, 0.0, 0.0) };

  // Knot vector.
  const std::vector<double> U = {0.0, 0.0, 0.0, 1, 2, 3, 4, 4, 5, 5, 5};

  // Degree.
  const int p = 2;

  // Construct B-curve.
  core_Ptr<t_bcurve> curve = new t_bcurve(Q, U, p);

  /* ==============
   *  Perform test
   * ============== */

  const double eps = 1e-7;
  const double u   = -1.0;
  //
  const t_xyz P_ref(-2.5, 0, 0);

  // Evaluate.
  t_xyz P;
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
  outcome res( DescriptionFn(), funcID );

  /* ======================
   *  Prepare input curve
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
  core_Ptr<t_bcurve> curve = t_bcurve::Instance(json);
  //
  if ( curve.IsNull() )
    return res.failure();

  /* ==============
   *  Perform test
   * ============== */

  const double eps = 1e-7;
  const double u   = 0.5;
  //
  const t_xyz P_ref(10.0, 0, 0);

  // Evaluate.
  t_xyz P;
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
  outcome res( DescriptionFn(), funcID );

  /* =====================
   *  Prepare input curve
   * ===================== */

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
  core_Ptr<t_bcurve> curve = t_bcurve::Instance(json);
  //
  if ( curve.IsNull() )
    return res.failure();

  /* ==============
   *  Perform test
   * ============== */

  const double eps = 1e-4;
  const double u   = 0.5;
  //
  const t_xyz P_ref(-70.5441, -6.21939, -30.8876);

  // Evaluate.
  t_xyz P;
  curve->Eval(u, P);

  // Check.
  if ( (P - P_ref).Modulus() > eps )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

mobius::outcome mobius::test_BSplineCurve::splitToBezier(const int funcID)
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
    continuity: C2,\
    domain: {\
        U_min: 0,\
        U_max: 43.5082523994\
    },\
    flags: {\
        is_rational: 0,\
        is_periodic: 0,\
        is_closed: 0\
    },\
    properties: {\
        degree: 5,\
        knots: [0, 0, 0, 0, 0, 0, 4.9136091691599999, 4.9136091691599999, 4.9136091691599999, 19.052154971499998, 19.052154971499998, 19.052154971499998, 30.250724551899999, 30.250724551899999, 30.250724551899999, 39.002968818399999, 39.002968818399999, 39.002968818399999, 43.5082523994, 43.5082523994, 43.5082523994, 43.5082523994, 43.5082523994, 43.5082523994],\
        num_poles: 18,\
        poles: [[-33.190276735200001, -53.844519367300002, 52.423727905200003], [-33.515300119899997, -54.460336051299997, 52.258211901700001], [-33.8325422518, -55.075785925200002, 52.083714055000002], [-34.142501742199997, -55.692059289699998, 51.899759320100003], [-35.309524008300002, -58.072170331400002, 51.153317583800003], [-36.349400456600002, -60.434301995200002, 50.270904322699998], [-37.046663655800003, -62.172300914799997, 49.540444420199997], [-38.157963430000002, -65.2555211066, 48.0942351391], [-39.007750568200002, -68.269584239400004, 46.3973330492], [-39.330273844799997, -69.584891405199997, 45.599001012800002], [-39.806148897900002, -71.895314042300001, 44.088184877000003], [-40.100323908299998, -74.127513756100001, 42.430353635300001], [-40.193472705600001, -75.088228240199996, 41.676797132099999], [-40.280013836199998, -76.517601654800004, 40.493350586699997], [-40.283222416599997, -77.899135308300004, 39.253251927800001], [-40.274649282299997, -78.362659474899999, 38.825633098099999], [-40.256423560199998, -78.820370015400002, 38.391697262400001], [-40.228596560299998, -79.272024098000003, 37.951724472199999]]\
    }\
  }";

  // Construct B-curve.
  core_Ptr<t_bcurve> curve = t_bcurve::Instance(json);
  //
  if ( curve.IsNull() )
    return res.failure();

  /* ==============
   *  Perform test
   * ============== */

  const int numSegmentsRef = 5;

  // Split to Bezier sergments.
  std::vector< t_ptr<t_bcurve> > segments;
  //
  if ( !curve->SplitToBezier(segments) )
    return res.failure();

  // Validate the number of segments.
  if ( segments.size() != numSegmentsRef )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

mobius::outcome mobius::test_BSplineCurve::makeBezier(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  // Construct B-curve.
  core_Ptr<t_bcurve>
    curve = t_bcurve::MakeBezier( 0, 1,
                                 {t_xyz(0., 0., 0.), t_xyz(1., 1., 0.), t_xyz(2., 1., 0.), t_xyz(3., 0., 0.)} );
  //
  if ( curve.IsNull() )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

mobius::outcome mobius::test_BSplineCurve::concatCompatible(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  // Construct B-curves.
  core_Ptr<t_bcurve>
    curves[2] = {
      t_bcurve::MakeBezier( 0, 1,
                           {t_xyz(0., 0., 0.), t_xyz(1., 1., 0.), t_xyz(2., 1., 0.), t_xyz(3., 0., 0.)} ),
      t_bcurve::MakeBezier( 1, 2,
                           {t_xyz(3., 0., 0.), t_xyz(4., 1., 0.), t_xyz(5., 1., 0.), t_xyz(6., 0., 0.)} )
  };

  // Concatenate.
  if ( !curves[0]->ConcatenateCompatible(curves[1]) )
    return res.failure();

  const int n = curves[0]->GetNumOfPoles() - 1;
  const int p = curves[0]->GetDegree();
  const int m = curves[0]->GetNumOfKnots() - 1;

  // Check validity.
  if ( !bspl::Check(n, m, p) )
    return res.failure();

  return res.success();
}
