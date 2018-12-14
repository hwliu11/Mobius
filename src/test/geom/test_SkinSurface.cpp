//-----------------------------------------------------------------------------
// Created on: 13 December 2018
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
#include <mobius/test_SkinSurface.h>

// Test includes
#include <mobius/test_CommonFacilities.h>

// Geom includes
#include <mobius/geom_InterpolateMultiCurve.h>
#include <mobius/geom_SkinSurface.h>

// Core includes
#include <mobius/core_Precision.h>

//-----------------------------------------------------------------------------

bool
  mobius::test_SkinSurface::runtest(const std::vector< ptr<bcurve> >& sections,
                                    const std::vector<xyz>&           D1lead,
                                    const std::vector<xyz>&           D1tail,
                                    const bool                        unify,
                                    const int                         degV,
                                    const ptr<bsurf>&                 surfRef)
{
  // Access common facilities.
  ptr<test_CommonFacilities> cf = test_CommonFacilities::Instance();

  // Skin ruled surface through the section curves.
  geom_SkinSurface skinner(sections, degV, unify);
  //
  skinner.AddLeadingTangencies(D1lead);
  skinner.AddTrailingTangencies(D1tail);
  //
  if ( !skinner.Perform() )
  {
    cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Cannot build skinning surface.");
    return false;
  }
  //
  const ptr<bsurf>& surfRes = skinner.GetResult();

  // Compare the constructed surface with the reference one.
  if ( !bsurf::Compare( surfRes,
                        surfRef,
                        core_Precision::Resolution2D(),
                        core_Precision::Resolution3D() ) )
  {
    cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Skinned surface deviates to much "
                                                             "from the expected surface.");
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------

//! Test scenario 001.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_SkinSurface::test001(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  // Access common facilities.
  ptr<test_CommonFacilities> cf = test_CommonFacilities::Instance();

  /* ======================
   *  Build section curves
   * ====================== */

  std::vector< ptr<bcurve> > sections;

  // Build curves.
  {
    std::vector<mobius::xyz> c0_pts = {
      mobius::xyz(0.,  0.,  0.),
      mobius::xyz(1.,  1.,  1.),
      mobius::xyz(1.,  2.,  2.),
      mobius::xyz(2.,  3.,  4.)
    };

    std::vector<mobius::xyz> c1_pts = {
      mobius::xyz(5.,  0.,  0.),
      mobius::xyz(5.,  1.,  2.),
      mobius::xyz(5.,  2.,  2.),
      mobius::xyz(6.,  3.,  5.)
    };

    std::vector<mobius::xyz> c2_pts = {
      mobius::xyz(9.,  0.,  0.),
      mobius::xyz(9.1, 1.,  2.),
      mobius::xyz(9.,  2.,  2.),
      mobius::xyz(9.1, 1.,  6.)
    };

    // Prepare points interpolator to build curves.
    geom_InterpolateMultiCurve multiInterp(3,
                                           ParamsSelection_Centripetal,
                                           KnotsSelection_Average);
    //
    multiInterp.AddRow(c0_pts);
    multiInterp.AddRow(c1_pts);
    multiInterp.AddRow(c2_pts);

    // Perform interpolation.
    if ( multiInterp.Perform() )
    {
      // Get section curves.
      for ( int k = 0; k < multiInterp.GetNumRows(); ++k )
        sections.push_back( multiInterp.GetResult(k) );
    }
    else
    {
      cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Multi-curve interpolation failed.");
      return res.failure();
    }
  }

  /* ===========================
   *  Prepare reference surface
   * =========================== */

  // JSON definition.
  std::string json =
  "{\
    entity: surface,\
    type: b-surface,\
    continuity: CN,\
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
        V_degree: 2,\
        U_knots: [0, 0, 0, 0, 1, 1, 1, 1],\
        V_knots: [0, 0, 0, 1, 1, 1],\
        num_poles_in_U_axis: 4,\
        num_poles_in_V_axis: 3,\
        poles: {\
            u0: [[4.3349144313197013e-016, -1.1946123722428327e-016, -2.123755328431703e-016], [5.1420608181621903, -6.1956169473235332e-017, 1.289579854722904e-016], [9, -2.256490036458685e-016, -1.0618776642158514e-016]],\
            u1: [[2.1380962197159086, 0.53135257815595094, 0.76318397092242618], [4.416621820964207, 0.78244515495858169, 6.1112650925611378], [9.3744839861275775, 0.067689792623005646, 4.7398552249647707]],\
            u2: [[-0.10957743023777558, 2.9369297198644349, 2.3066278087235697], [4.1009494003057654, 2.254260607871462, -1.9651136832015064], [8.6843915419660114, 4.1975335421461653, -1.4797586827572815]],\
            u3: [[1.9999999999999978, 2.9999999999999996, 3.9999999999999991], [6.167802790819561, 4.0830827257961566, 4.9201038910528254], [9.0999999999999961, 1, 6]]\
        }\
    }\
  }";

  // Construct reference B-surface.
  core_Ptr<bsurf> surfRef = bsurf::Instance(json);
  //
  if ( surfRef.IsNull() )
    return res.failure();

  /* ==============
   *  Perform test
   * ============== */

  if ( !runtest(sections, std::vector<xyz>(), std::vector<xyz>(), false, 2, surfRef) )
    return res.failure();

  // Report execution time.
  SetVarDescr("time", res.time(), ID(), funcID);

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 002. Tests that skinning results good error code when
//! the user specifies wrong V degree.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_SkinSurface::test002(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  // Access common facilities.
  ptr<test_CommonFacilities> cf = test_CommonFacilities::Instance();

  /* ======================
   *  Build section curves
   * ====================== */

  std::vector< ptr<bcurve> > sections;

  // Tangency constraints.
  std::vector<mobius::xyz> c0_D1, c1_D1;

  // Build curves.
  {
    std::vector<mobius::xyz> c0_pts = {
      mobius::xyz(0.,  0.,  0.),
      mobius::xyz(1.,  1.,  1.),
      mobius::xyz(1.,  2.,  2.)
    };
    //
    c0_D1 = {
      mobius::xyz(0.,  0.,  3.),
      mobius::xyz(0.,  0.,  3.),
      mobius::xyz(0.,  0.,  3.)
    };

    std::vector<mobius::xyz> c1_pts = {
      mobius::xyz(5.,  0.,  0.),
      mobius::xyz(5.,  1.,  2.),
      mobius::xyz(5.,  2.,  2.)
    };
    //
    c1_D1 = {
      mobius::xyz(0.,  0.,  -3.),
      mobius::xyz(0.,  0.,  -3.),
      mobius::xyz(0.,  0.,  -3.)
    };

    // Prepare points interpolator to build curves.
    geom_InterpolateMultiCurve multiInterp(2,
                                           ParamsSelection_Centripetal,
                                           KnotsSelection_Average);
    //
    multiInterp.AddRow(c0_pts);
    multiInterp.AddRow(c1_pts);

    // Perform interpolation.
    if ( multiInterp.Perform() )
    {
      // Get section curves.
      for ( int k = 0; k < multiInterp.GetNumRows(); ++k )
        sections.push_back( multiInterp.GetResult(k) );
    }
    else
    {
      cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Multi-curve interpolation failed.");
      return res.failure();
    }
  }

  /* ==============
   *  Perform test
   * ============== */

  // Skin ruled surface through the section curves.
  geom_SkinSurface skinner(sections, 1, false);
  //
  skinner.AddLeadingTangencies(c0_D1);
  skinner.AddTrailingTangencies(c1_D1);
  //
  if ( skinner.Perform() )
  {
    cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Unexpected success status.");
    return res.failure();
  }
  //
  if ( skinner.GetErrorCode() != geom_SkinSurface::ErrCode_BadVDegree)
  {
    cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "ErrCode_BadVDegree is expected.");
    return res.failure();
  }

  // Report execution time.
  SetVarDescr("time", res.time(), ID(), funcID);

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 003. Performs skinning with tangency constraints.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_SkinSurface::test003(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  // Access common facilities.
  ptr<test_CommonFacilities> cf = test_CommonFacilities::Instance();

  /* ======================
   *  Build section curves
   * ====================== */

  std::vector< ptr<bcurve> > sections;

  // Tangency constraints.
  std::vector<mobius::xyz> c0_D1, c1_D1;

  // Build curves.
  {
    std::vector<mobius::xyz> c0_pts = {
      mobius::xyz(0.,  0.,  0.),
      mobius::xyz(1.,  1.,  1.),
      mobius::xyz(1.,  2.,  2.)
    };
    //
    c0_D1 = {
      mobius::xyz(0.,  0.,  3.),
      mobius::xyz(0.,  0.,  3.),
      mobius::xyz(0.,  0.,  3.)
    };

    std::vector<mobius::xyz> c1_pts = {
      mobius::xyz(5.,  0.,  0.),
      mobius::xyz(5.,  1.,  2.),
      mobius::xyz(5.,  2.,  2.)
    };
    //
    c1_D1 = {
      mobius::xyz(0.,  0.,  -3.),
      mobius::xyz(0.,  0.,  -3.),
      mobius::xyz(0.,  0.,  -3.)
    };

    // Prepare points interpolator to build curves.
    geom_InterpolateMultiCurve multiInterp(2,
                                           ParamsSelection_Centripetal,
                                           KnotsSelection_Average);
    //
    multiInterp.AddRow(c0_pts);
    multiInterp.AddRow(c1_pts);

    // Perform interpolation.
    if ( multiInterp.Perform() )
    {
      // Get section curves.
      for ( int k = 0; k < multiInterp.GetNumRows(); ++k )
        sections.push_back( multiInterp.GetResult(k) );
    }
    else
    {
      cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Multi-curve interpolation failed.");
      return res.failure();
    }
  }

  /* ===========================
   *  Prepare reference surface
   * =========================== */

  // JSON definition.
  std::string json =
  "{\
    entity: surface,\
    type: b-surface,\
    continuity: C1,\
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
        U_degree: 2,\
        V_degree: 2,\
        U_knots: [0, 0, 0, 1, 1, 1],\
        V_knots: [0, 0, 0, 0.5, 1, 1, 1],\
        num_poles_in_U_axis: 3,\
        num_poles_in_V_axis: 4,\
        poles: {\
            u0: [[-8.6014429388883521e-016, 6.9624504123215511e-016, 7.8504622934188746e-016], [-1.8841109504205307e-015, 6.962450412321553e-016, 0.75000000000000078], [5.0000000000000009, 6.962450412321554e-016, 0.75000000000000022], [5, 6.962450412321554e-016, 2.3551386880256624e-016]],\
            u1: [[1.3892253635443865, 0.74692395306220616, 0.74692395306220583], [1.3892253635443863, 0.74692395306220605, 1.4969239530622054], [4.9999999999999982, 0.7469239530622066, 3.5284507270887753], [4.9999999999999973, 0.74692395306220649, 2.7784507270887748]],\
            u2: [[0.99999999999999889, 1.9999999999999993, 1.9999999999999991], [0.99999999999999822, 1.9999999999999993, 2.7499999999999991], [5.0000000000000009, 1.9999999999999998, 2.7499999999999996], [5, 1.9999999999999998, 1.9999999999999998]]\
        }\
    }\
  }";

  // Construct reference B-surface.
  core_Ptr<bsurf> surfRef = bsurf::Instance(json);
  //
  if ( surfRef.IsNull() )
    return res.failure();

  /* ==============
   *  Perform test
   * ============== */

  if ( !runtest(sections, c0_D1, c1_D1, false, 2, surfRef) )
    return res.failure();

  // Report execution time.
  SetVarDescr("time", res.time(), ID(), funcID);

  return res.success();
}
