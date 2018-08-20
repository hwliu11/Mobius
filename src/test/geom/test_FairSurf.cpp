//-----------------------------------------------------------------------------
// Created on: 20 August 2018
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
#include <mobius/test_FairSurf.h>

// Test includes
#include <mobius/test_CommonFacilities.h>

// Geom includes
#include <mobius/geom_FairBSurf.h>

//-----------------------------------------------------------------------------

//! Test scenario 001. This test function checks that mapping of indices is
//! done correctly.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_FairSurf::testIndices(const int funcID)
{
  outcome res( DescriptionFn() );

  // JSON definition.
  std::string json =
  "{\
    entity: surface,\
    type: b-surface,\
    continuity: C2,\
    domain: {\
        U_min: 0,\
        U_max: 1,\
        V_min: 0,\
        V_max: 1\
    },\
    flags: {\
        is_U_rational: 0,\
        is_V_rational: 0,\
        is_U_periodic: 0,\
        is_V_periodic: 0,\
        is_U_closed: 0,\
        is_V_closed: 0\
    },\
    properties: {\
        U_degree: 3,\
        V_degree: 1,\
        U_knots: [0, 0, 0, 0, 0.41551883124182798, 0.59974028777161115, 1, 1, 1, 1],\
        V_knots: [0, 0, 1, 1],\
        num_poles_in_U_axis: 6,\
        num_poles_in_V_axis: 2,\
        poles: {\
            u0: [[241.87747035573125, 62.485923112593042, 0], [307.49011857707512, 49.837701768719491, 0]],\
            u1: [[245.29391602546553, 57.035146252318057, 0], [306.57174310419981, 45.021517271473421, 0]],\
            u2: [[250.62433050286035, 49.413132207145885, 25], [303.8174241480333, 37.906450425886014, 0]],\
            u3: [[260.43129359394032, 37.406082628166004, -22], [295.65540308364263, 29.098297811030434, -30]],\
            u4: [[266.65853554251112, 30.811365428505532, 0], [288.81855843279288, 26.561552469297354, 0]],\
            u5: [[271.12648221343875, 26.517543665952715, 0], [284.16996047430831, 25.727029831960614, 0]]\
        }\
    }\
  }";

  // Construct B-surface.
  core_Ptr<bsurf> surf = bsurf::Instance(json);
  //
  if ( surf.IsNull() )
    return res.failure();

  geom_FairBSurf F(surf, 0.0, NULL, NULL);

  // Test 1.
  {
    int i = 0, j = 0;
    F.GetIJ(0, i, j);
    //
    if ( i != 0 || j != 0 )
      return res.failure();
  }

  // Test 2.
  {
    int i = 0, j = 0;
    F.GetIJ(1, i, j);
    //
    if ( i != 0 || j != 1 )
      return res.failure();
  }

  // Test 3.
  {
    int i = 0, j = 0;
    F.GetIJ(2, i, j);
    //
    if ( i != 1 || j != 0 )
      return res.failure();
  }

  // Test 4.
  {
    int i = 0, j = 0;
    F.GetIJ(3, i, j);
    //
    if ( i != 1 || j != 1 )
      return res.failure();
  }

  // Test 5.
  {
    int i = 0, j = 0;
    F.GetIJ(4, i, j);
    //
    if ( i != 2 || j != 0 )
      return res.failure();
  }

  // Test 6.
  {
    if ( F.GetK(0, 0) != 0 )
      return res.failure();
  }

  // Test 7.
  {
    if ( F.GetK(0, 1) != 1 )
      return res.failure();
  }

  // Test 8.
  {
    if ( F.GetK(1, 0) != 2 )
      return res.failure();
  }

  // Test 9.
  {
    if ( F.GetK(1, 1) != 3 )
      return res.failure();
  }

  // Test 10.
  {
    if ( F.GetK(2, 0) != 4 )
      return res.failure();
  }

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 002.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_FairSurf::testFairing01(const int funcID)
{
  outcome res( DescriptionFn() );

  // Access common facilities.
  ptr<test_CommonFacilities> cf = test_CommonFacilities::Instance();

  // JSON definition.
  std::string json =
  "{\
    entity: surface,\
    type: b-surface,\
    continuity: C2,\
    domain: {\
        U_min: 0,\
        U_max: 1,\
        V_min: 0,\
        V_max: 1\
    },\
    flags: {\
        is_U_rational: 0,\
        is_V_rational: 0,\
        is_U_periodic: 0,\
        is_V_periodic: 0,\
        is_U_closed: 0,\
        is_V_closed: 0\
    },\
    properties: {\
        U_degree: 3,\
        V_degree: 1,\
        U_knots: [0, 0, 0, 0, 0.41551883124182798, 0.59974028777161115, 1, 1, 1, 1],\
        V_knots: [0, 0, 1, 1],\
        num_poles_in_U_axis: 6,\
        num_poles_in_V_axis: 2,\
        poles: {\
            u0: [[241.87747035573125, 62.485923112593042, 0], [307.49011857707512, 49.837701768719491, 0]],\
            u1: [[245.29391602546553, 57.035146252318057, 0], [306.57174310419981, 45.021517271473421, 0]],\
            u2: [[250.62433050286035, 49.413132207145885, 25], [303.8174241480333, 37.906450425886014, 0]],\
            u3: [[260.43129359394032, 37.406082628166004, -22], [295.65540308364263, 29.098297811030434, -30]],\
            u4: [[266.65853554251112, 30.811365428505532, 0], [288.81855843279288, 26.561552469297354, 0]],\
            u5: [[271.12648221343875, 26.517543665952715, 0], [284.16996047430831, 25.727029831960614, 0]]\
        }\
    }\
  }";

  // Construct B-surface.
  core_Ptr<bsurf> surf = bsurf::Instance(json);
  //
  if ( surf.IsNull() )
    return res.failure();

  cf->ProgressNotifier.SendLogMessage( MobiusInfo(Normal) << "Initial bending energy: %1"
                                                          << surf->ComputeBendingEnergy() );

  // Perform fairing.
  geom_FairBSurf F(surf, 1.0, NULL, NULL);
  //
  if ( !F.Perform() )
    return res.failure();

  // Get the result.
  const core_Ptr<bsurf>& result = F.GetResult();

  cf->ProgressNotifier.SendLogMessage( MobiusInfo(Normal) << "Resulting bending energy: %1"
                                                          << result->ComputeBendingEnergy() );

  return res.success();
}
