//-----------------------------------------------------------------------------
// Created on: 10 December 2018
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
#include <mobius/test_Decompose.h>

// Geom includes
#include <mobius/geom_BSplineCurve.h>

// bspl includes
#include <mobius/bspl_Decompose.h>

//-----------------------------------------------------------------------------

bool
  mobius::test_Decompose::decomposeCurve(const ptr<bcurve>&      curve,
                                         const int               numSegmentsRef,
                                         const std::vector<int>& breakpointsRef)
{
  // Input arguments.
  const int                  n  = curve->GetNumOfPoles() - 1;
  const int                  p  = curve->GetDegree();
  const std::vector<double>& U  = curve->GetKnots();
  const std::vector<xyz>&    Pw = curve->GetPoles();

  // Output arguments.
  int nb = 0;
  std::vector< std::vector<xyz> > Qw;
  std::vector<int> breakpoints;

  // Perform curve decomposition.
  bspl_Decompose decomposer;
  //
  if ( !decomposer(n, p, U, Pw, nb, Qw, breakpoints) )
    return false;

  // Check the expected number of segments.
  if ( Qw.size() != numSegmentsRef )
    return false;

  // Check the indices of the breakpoints.
  if ( breakpoints != breakpointsRef )
    return false;

  return true;
}

//-----------------------------------------------------------------------------

mobius::outcome mobius::test_Decompose::test01(const int funcID)
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
  ptr<bcurve> curve = bcurve::Instance(json);
  //
  if ( curve.IsNull() )
    return res.failure();

  /* =================================
   *  Perform decomposition algorithm
   * ================================= */

  // Expected number of segments.
  const int refNumSegments = 5;

  // Expected breakpoints.
  std::vector<int> breakpoints = {6, 9, 12, 15, 18};

  // Perform test.
  if ( !decomposeCurve(curve, refNumSegments, breakpoints) )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

mobius::outcome mobius::test_Decompose::test02(const int funcID)
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

  /* =================================
   *  Perform decomposition algorithm
   * ================================= */

  // Expected number of segments.
  const int refNumSegments = 1;

  // Expected breakpoints.
  std::vector<int> breakpoints = {4};

  // Perform test.
  if ( !decomposeCurve(curve, refNumSegments, breakpoints) )
    return res.failure();

  return res.success();
}
