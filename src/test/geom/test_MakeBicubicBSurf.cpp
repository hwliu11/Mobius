//-----------------------------------------------------------------------------
// Created on: 14 December 2018
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
#include <mobius/test_MakeBicubicBSurf.h>

// Test includes
#include <mobius/test_CommonFacilities.h>

// Geom includes
#include <mobius/geom_MakeBicubicBSurf.h>

// Core includes
#include <mobius/core_Precision.h>

//-----------------------------------------------------------------------------

bool mobius::test_MakeBicubicBSurf::verifyConstraints(const t_xyz&          S00,
                                                      const t_xyz&          S01,
                                                      const t_xyz&          S10,
                                                      const t_xyz&          S11,
                                                      const t_xyz&          dS_du00,
                                                      const t_xyz&          dS_du01,
                                                      const t_xyz&          dS_du10,
                                                      const t_xyz&          dS_du11,
                                                      const t_xyz&          dS_dv00,
                                                      const t_xyz&          dS_dv01,
                                                      const t_xyz&          dS_dv10,
                                                      const t_xyz&          dS_dv11,
                                                      const t_xyz&          d2S_dudv00,
                                                      const t_xyz&          d2S_dudv01,
                                                      const t_xyz&          d2S_dudv10,
                                                      const t_xyz&          d2S_dudv11,
                                                      const t_ptr<t_bsurf>& resSurf)
{
  // Precision to compare Cartesian coordinates.
  const double prec = core_Precision::Resolution3D();

  // Access common facilities.
  t_ptr<test_CommonFacilities> cf = test_CommonFacilities::Instance();

  // (u,v) = (0,0):
  {
    t_xyz S, dS_dU, dS_dV, d2S_dU2, d2S_dV2, d2S_dUV;
    resSurf->Eval_D2(0., 0., S, dS_dU, dS_dV, d2S_dU2, d2S_dV2, d2S_dUV);

    // Primal.
    if ( (S - S00).Modulus() > prec )
    {
      cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Violated S00 constraint.");
      return false;
    }

    // Partial by U.
    if ( (dS_dU - dS_du00).Modulus() > prec )
    {
      cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Violated dS_du00 constraint.");
      return false;
    }

    // Partial by V.
    if ( (dS_dV - dS_dv00).Modulus() > prec )
    {
      cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Violated dS_dv00 constraint.");
      return false;
    }

    // Partial by UV.
    if ( (d2S_dUV - d2S_dudv00).Modulus() > prec )
    {
      cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Violated d2S_dudv00 constraint.");
      return false;
    }
  }

  // (u,v) = (0,1):
  {
    t_xyz S, dS_dU, dS_dV, d2S_dU2, d2S_dV2, d2S_dUV;
    resSurf->Eval_D2(0., 1., S, dS_dU, dS_dV, d2S_dU2, d2S_dV2, d2S_dUV);

    // Primal.
    if ( (S - S01).Modulus() > prec )
    {
      cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Violated S01 constraint.");
      return false;
    }

    // Partial by U.
    if ( (dS_dU - dS_du01).Modulus() > prec )
    {
      cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Violated dS_du01 constraint.");
      return false;
    }

    // Partial by V.
    if ( (dS_dV - dS_dv01).Modulus() > prec )
    {
      cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Violated dS_dv01 constraint.");
      return false;
    }

    // Partial by UV.
    if ( (d2S_dUV - d2S_dudv01).Modulus() > prec )
    {
      cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Violated d2S_dudv01 constraint.");
      return false;
    }
  }

  // (u,v) = (1,0):
  {
    t_xyz S, dS_dU, dS_dV, d2S_dU2, d2S_dV2, d2S_dUV;
    resSurf->Eval_D2(1., 0., S, dS_dU, dS_dV, d2S_dU2, d2S_dV2, d2S_dUV);

    // Primal.
    if ( (S - S10).Modulus() > prec )
    {
      cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Violated S10 constraint.");
      return false;
    }

    // Partial by U.
    if ( (dS_dU - dS_du10).Modulus() > prec )
    {
      cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Violated dS_du10 constraint.");
      return false;
    }

    // Partial by V.
    if ( (dS_dV - dS_dv10).Modulus() > prec )
    {
      cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Violated dS_dv10 constraint.");
      return false;
    }

    // Partial by UV.
    if ( (d2S_dUV - d2S_dudv10).Modulus() > prec )
    {
      cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Violated d2S_dudv10 constraint.");
      return false;
    }
  }

  // (u,v) = (1,1):
  {
    t_xyz S, dS_dU, dS_dV, d2S_dU2, d2S_dV2, d2S_dUV;
    resSurf->Eval_D2(1., 1., S, dS_dU, dS_dV, d2S_dU2, d2S_dV2, d2S_dUV);

    // Primal.
    if ( (S - S11).Modulus() > prec )
    {
      cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Violated S11 constraint.");
      return false;
    }

    // Partial by U.
    if ( (dS_dU - dS_du11).Modulus() > prec )
    {
      cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Violated dS_du11 constraint.");
      return false;
    }

    // Partial by V.
    if ( (dS_dV - dS_dv11).Modulus() > prec )
    {
      cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Violated dS_dv11 constraint.");
      return false;
    }

    // Partial by UV.
    if ( (d2S_dUV - d2S_dudv11).Modulus() > prec )
    {
      cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Violated d2S_dudv11 constraint.");
      return false;
    }
  }

  return true;
}

//-----------------------------------------------------------------------------

mobius::outcome
  mobius::test_MakeBicubicBSurf::performTest(const int    funcID,
                                             const t_xyz& S00,
                                             const t_xyz& S01,
                                             const t_xyz& S10,
                                             const t_xyz& S11,
                                             const t_xyz& dS_du00,
                                             const t_xyz& dS_du01,
                                             const t_xyz& dS_du10,
                                             const t_xyz& dS_du11,
                                             const t_xyz& dS_dv00,
                                             const t_xyz& dS_dv01,
                                             const t_xyz& dS_dv10,
                                             const t_xyz& dS_dv11,
                                             const t_xyz& d2S_dudv00,
                                             const t_xyz& d2S_dudv01,
                                             const t_xyz& d2S_dudv10,
                                             const t_xyz& d2S_dudv11)
{
  outcome res( DescriptionFn(), funcID );

  // Access common facilities.
  t_ptr<test_CommonFacilities> cf = test_CommonFacilities::Instance();

  /* =============
   *  Build patch
   * ============= */

  // Build constrained bicubic patch.
  geom_MakeBicubicBSurf mkBicubic(S00, S01, S10, S11, // Pin points.
                                  dS_du00, dS_du01, dS_du10, dS_du11,
                                  dS_dv00, dS_dv01, dS_dv10, dS_dv11,
                                  d2S_dudv00, d2S_dudv01, d2S_dudv10, d2S_dudv11,
                                  cf->ProgressNotifier, nullptr);
  //
  if ( !mkBicubic.Perform() )
  {
    cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Failed to make a bicubic patch.");
    return res.failure();
  }

  // Report execution time.
  SetVarDescr("time", res.time(), ID(), funcID);

  // Get result.
  const t_ptr<t_bsurf>& resSurf = mkBicubic.GetResult();

  /* ============================================================
   *  Evaluate surface to check if the constraints are satisfied
   * ============================================================ */

  if ( !verifyConstraints(S00, S01, S10, S11,
                          dS_du00, dS_du01, dS_du10, dS_du11,
                          dS_dv00, dS_dv01, dS_dv10, dS_dv11,
                          d2S_dudv00, d2S_dudv01, d2S_dudv10, d2S_dudv11,
                          resSurf) )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

mobius::outcome
  mobius::test_MakeBicubicBSurf::test001(const int funcID)
{
  // Define constraints.
  const t_xyz S00        = t_xyz(0., 0., 0.);
  const t_xyz S01        = t_xyz(3., 0., 0.);
  const t_xyz S10        = t_xyz(0., 3., 0.);
  const t_xyz S11        = t_xyz(4., 3., 0.);
  const t_xyz dS_du00    = t_xyz(0., 1., 1.);
  const t_xyz dS_du01    = t_xyz(0., 0., 0.);
  const t_xyz dS_du10    = t_xyz(0., 0., 0.);
  const t_xyz dS_du11    = t_xyz(0., 0., 1.);
  const t_xyz dS_dv00    = t_xyz(1., 0., 0.);
  const t_xyz dS_dv01    = t_xyz(0., 0., 0.);
  const t_xyz dS_dv10    = t_xyz(0., 0., 0.);
  const t_xyz dS_dv11    = t_xyz(0., 0., 3.);
  const t_xyz d2S_dudv00 = t_xyz(0., 0., 0.);
  const t_xyz d2S_dudv01 = t_xyz(0., 0., 0.);
  const t_xyz d2S_dudv10 = t_xyz(0., 0., 0.);
  const t_xyz d2S_dudv11 = t_xyz(0., 0., 0.);

  // Perform test.
  return performTest(funcID,
                     S00, S01, S10, S11,
                     dS_du00, dS_du01, dS_du10, dS_du11,
                     dS_dv00, dS_dv01, dS_dv10, dS_dv11,
                     d2S_dudv00, d2S_dudv01, d2S_dudv10, d2S_dudv11);
}

//-----------------------------------------------------------------------------

mobius::outcome
  mobius::test_MakeBicubicBSurf::test002(const int funcID)
{
  // Define constraints.
  const t_xyz S00        = t_xyz(0., 0., 0.);
  const t_xyz S01        = t_xyz(3., 0., 0.);
  const t_xyz S10        = t_xyz(0., 3., 0.);
  const t_xyz S11        = t_xyz(4., 3., 0.);
  const t_xyz dS_du00    = t_xyz(0., 1., 1.);
  const t_xyz dS_du01    = t_xyz(0., 0., 0.);
  const t_xyz dS_du10    = t_xyz(0., 0., 0.);
  const t_xyz dS_du11    = t_xyz(0., 0., 1.);
  const t_xyz dS_dv00    = t_xyz(1., 0., 0.);
  const t_xyz dS_dv01    = t_xyz(0., 0., 0.);
  const t_xyz dS_dv10    = t_xyz(0., 0., 0.);
  const t_xyz dS_dv11    = t_xyz(0., 0., 3.);
  const t_xyz d2S_dudv00 = t_xyz(0., 0., 0.);
  const t_xyz d2S_dudv01 = t_xyz(0., 0., 0.);
  const t_xyz d2S_dudv10 = t_xyz(0., 0., 0.);
  const t_xyz d2S_dudv11 = t_xyz(1., 1., 0.);

  // Perform test.
  return performTest(funcID,
                     S00, S01, S10, S11,
                     dS_du00, dS_du01, dS_du10, dS_du11,
                     dS_dv00, dS_dv01, dS_dv10, dS_dv11,
                     d2S_dudv00, d2S_dudv01, d2S_dudv10, d2S_dudv11);
}

//-----------------------------------------------------------------------------

mobius::outcome
  mobius::test_MakeBicubicBSurf::test003(const int funcID)
{
  // Define constraints.
  const t_xyz S00        = t_xyz(0., 0., 0.);
  const t_xyz S01        = t_xyz(3., 0., 0.);
  const t_xyz S10        = t_xyz(0., 3., 0.);
  const t_xyz S11        = t_xyz(4., 3., 0.);
  const t_xyz dS_du00    = t_xyz(0., 1., 1.);
  const t_xyz dS_du01    = t_xyz(0., 0., 1.);
  const t_xyz dS_du10    = t_xyz(0., 0., 1.);
  const t_xyz dS_du11    = t_xyz(0., 0., 1.);
  const t_xyz dS_dv00    = t_xyz(1., 0., 0.);
  const t_xyz dS_dv01    = t_xyz(0., 0., 0.);
  const t_xyz dS_dv10    = t_xyz(0., 0., 0.);
  const t_xyz dS_dv11    = t_xyz(0., 0., 3.);
  const t_xyz d2S_dudv00 = t_xyz(0., 1., 1.);
  const t_xyz d2S_dudv01 = t_xyz(1., 1., 0.);
  const t_xyz d2S_dudv10 = t_xyz(1., 1., 1.);
  const t_xyz d2S_dudv11 = t_xyz(1., 1., 0.);

  // Perform test.
  return performTest(funcID,
                     S00, S01, S10, S11,
                     dS_du00, dS_du01, dS_du10, dS_du11,
                     dS_dv00, dS_dv01, dS_dv10, dS_dv11,
                     d2S_dudv00, d2S_dudv01, d2S_dudv10, d2S_dudv11);
}

//-----------------------------------------------------------------------------

mobius::outcome
  mobius::test_MakeBicubicBSurf::test004(const int funcID)
{
  // Define constraints.
  const t_xyz S00        = t_xyz(0., 0., 0.);
  const t_xyz S01        = t_xyz(3., 0., 0.);
  const t_xyz S10        = t_xyz(0., 3., 0.);
  const t_xyz S11        = t_xyz(4., 3., 0.);
  const t_xyz dS_du00    = t_xyz(0., 0., 10.);
  const t_xyz dS_du01    = t_xyz(0., 0., -10.);
  const t_xyz dS_du10    = t_xyz(0., 0., 10.);
  const t_xyz dS_du11    = t_xyz(0., 0., -10.);
  const t_xyz dS_dv00    = t_xyz(0., 0., 10.);
  const t_xyz dS_dv01    = t_xyz(0., 0., -10.);
  const t_xyz dS_dv10    = t_xyz(0., 0., 10.);
  const t_xyz dS_dv11    = t_xyz(0., 0., -10.);
  const t_xyz d2S_dudv00 = t_xyz(0., 0., 0.);
  const t_xyz d2S_dudv01 = t_xyz(0., 0., 0.);
  const t_xyz d2S_dudv10 = t_xyz(0., 0., 0.);
  const t_xyz d2S_dudv11 = t_xyz(0., 0., 0.);

  // Perform test.
  return performTest(funcID,
                     S00, S01, S10, S11,
                     dS_du00, dS_du01, dS_du10, dS_du11,
                     dS_dv00, dS_dv01, dS_dv10, dS_dv11,
                     d2S_dudv00, d2S_dudv01, d2S_dudv10, d2S_dudv11);
}

//-----------------------------------------------------------------------------

mobius::outcome
  mobius::test_MakeBicubicBSurf::test005(const int funcID)
{
  // Define constraints.
  const t_xyz S00        = t_xyz(0., 0., 0.);
  const t_xyz S01        = t_xyz(3., 0., 0.);
  const t_xyz S10        = t_xyz(0., 3., 0.);
  const t_xyz S11        = t_xyz(4., 3., 0.);
  const t_xyz dS_du00    = t_xyz(0., 0., 0.);
  const t_xyz dS_du01    = t_xyz(0., 0., 0.);
  const t_xyz dS_du10    = t_xyz(0., 0., 0.);
  const t_xyz dS_du11    = t_xyz(0., 0., 0.);
  const t_xyz dS_dv00    = t_xyz(0., 0., 0.);
  const t_xyz dS_dv01    = t_xyz(0., 0., 0.);
  const t_xyz dS_dv10    = t_xyz(0., 0., 0.);
  const t_xyz dS_dv11    = t_xyz(0., 0., 0.);
  const t_xyz d2S_dudv00 = t_xyz(0., 0., 0.);
  const t_xyz d2S_dudv01 = t_xyz(0., 0., 0.);
  const t_xyz d2S_dudv10 = t_xyz(0., 0., 0.);
  const t_xyz d2S_dudv11 = t_xyz(0., 0., 0.);

  // Perform test.
  return performTest(funcID,
                     S00, S01, S10, S11,
                     dS_du00, dS_du01, dS_du10, dS_du11,
                     dS_dv00, dS_dv01, dS_dv10, dS_dv11,
                     d2S_dudv00, d2S_dudv01, d2S_dudv10, d2S_dudv11);
}

//-----------------------------------------------------------------------------

mobius::outcome
  mobius::test_MakeBicubicBSurf::test006(const int funcID)
{
  // Define constraints.
  const t_xyz S00        = t_xyz(0., 0., 0.);
  const t_xyz S01        = t_xyz(3., 0., 0.);
  const t_xyz S10        = t_xyz(0., 3., 0.);
  const t_xyz S11        = t_xyz(4., 3., 0.);
  const t_xyz dS_du00    = t_xyz(0., 0., 0.);
  const t_xyz dS_du01    = t_xyz(0., 0., 0.);
  const t_xyz dS_du10    = t_xyz(0., 0., 0.);
  const t_xyz dS_du11    = t_xyz(0., 0., 0.);
  const t_xyz dS_dv00    = t_xyz(0., 0., 0.);
  const t_xyz dS_dv01    = t_xyz(0., 0., 0.);
  const t_xyz dS_dv10    = t_xyz(0., 0., 0.);
  const t_xyz dS_dv11    = t_xyz(0., 0., 0.);
  const t_xyz d2S_dudv00 = t_xyz(0., 0., 0.);
  const t_xyz d2S_dudv01 = t_xyz(0., 0., 0.);
  const t_xyz d2S_dudv10 = t_xyz(0., 0., 0.);
  const t_xyz d2S_dudv11 = t_xyz(0., 0., 10.);

  // Perform test.
  return performTest(funcID,
                     S00, S01, S10, S11,
                     dS_du00, dS_du01, dS_du10, dS_du11,
                     dS_dv00, dS_dv01, dS_dv10, dS_dv11,
                     d2S_dudv00, d2S_dudv01, d2S_dudv10, d2S_dudv11);
}

//-----------------------------------------------------------------------------

mobius::outcome
  mobius::test_MakeBicubicBSurf::test007(const int funcID)
{
  // Define constraints.
  const t_xyz S00        = t_xyz(0., 0., 0.);
  const t_xyz S01        = t_xyz(3., 0., 0.);
  const t_xyz S10        = t_xyz(0., 3., 0.);
  const t_xyz S11        = t_xyz(4., 3., 0.);
  const t_xyz dS_du00    = t_xyz(0., 0., 0.);
  const t_xyz dS_du01    = t_xyz(0., 0., 0.);
  const t_xyz dS_du10    = t_xyz(0., 0., 0.);
  const t_xyz dS_du11    = t_xyz(0., 0., 0.);
  const t_xyz dS_dv00    = t_xyz(0., 0., 0.);
  const t_xyz dS_dv01    = t_xyz(0., 0., 0.);
  const t_xyz dS_dv10    = t_xyz(0., 0., 0.);
  const t_xyz dS_dv11    = t_xyz(0., 0., 0.);
  const t_xyz d2S_dudv00 = t_xyz(0., 0., 0.);
  const t_xyz d2S_dudv01 = t_xyz(0., 0., 0.);
  const t_xyz d2S_dudv10 = t_xyz(0., 0., 0.);
  const t_xyz d2S_dudv11 = t_xyz(0., 0., 50.);

  // Perform test.
  return performTest(funcID,
                     S00, S01, S10, S11,
                     dS_du00, dS_du01, dS_du10, dS_du11,
                     dS_dv00, dS_dv01, dS_dv10, dS_dv11,
                     d2S_dudv00, d2S_dudv01, d2S_dudv10, d2S_dudv11);
}

//-----------------------------------------------------------------------------

mobius::outcome
  mobius::test_MakeBicubicBSurf::test008(const int funcID)
{
  // Define constraints.
  const t_xyz S00        = t_xyz(0., 0., 0.);
  const t_xyz S01        = t_xyz(3., 0., 0.);
  const t_xyz S10        = t_xyz(0., 3., 0.);
  const t_xyz S11        = t_xyz(4., 3., 0.);
  const t_xyz dS_du00    = t_xyz(0., 1., 0.);
  const t_xyz dS_du01    = t_xyz(0., 5., 0.);
  const t_xyz dS_du10    = t_xyz(5., 0., 0.);
  const t_xyz dS_du11    = t_xyz(1., 1., 1.);
  const t_xyz dS_dv00    = t_xyz(1., 0., 0.);
  const t_xyz dS_dv01    = t_xyz(0., 5., 0.);
  const t_xyz dS_dv10    = t_xyz(5., 0., 0.);
  const t_xyz dS_dv11    = t_xyz(1., 1., 1.);
  const t_xyz d2S_dudv00 = t_xyz(0., 0., 0.);
  const t_xyz d2S_dudv01 = t_xyz(0., 0., 0.);
  const t_xyz d2S_dudv10 = t_xyz(0., 0., 0.);
  const t_xyz d2S_dudv11 = t_xyz(0., 0., 0.);

  // Perform test.
  return performTest(funcID,
                     S00, S01, S10, S11,
                     dS_du00, dS_du01, dS_du10, dS_du11,
                     dS_dv00, dS_dv01, dS_dv10, dS_dv11,
                     d2S_dudv00, d2S_dudv01, d2S_dudv10, d2S_dudv11);
}
