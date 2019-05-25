//-----------------------------------------------------------------------------
// Created on: 25 May 2019
//-----------------------------------------------------------------------------
// Copyright (c) 2019-present, Sergey Slyadnev
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
#include <mobius/test_PositionCloud.h>

// Test includes
#include <mobius/test_CommonFacilities.h>

// Core includes
#include <mobius/core_Precision.h>

// Geom includes
#include <mobius/geom_BuildAveragePlane.h>
#include <mobius/geom_PositionCloud.h>

#undef FILE_DEBUG
#if defined FILE_DEBUG
  #pragma message("===== warning: FILE_DEBUG is enabled")
#endif

//-----------------------------------------------------------------------------

// Filenames are specified relatively to MOBIUS_TEST_DATA environment variable.
#define filename_points_001 "points/001_points.asc"

//-----------------------------------------------------------------------------

//! Test scenario 001: build average plane for the whole point cloud.
//! \param[in] funcID function ID.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_PositionCloud::buildAveragePlane01(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  // Access common facilities.
  ptr<test_CommonFacilities> cf = test_CommonFacilities::Instance();

  // File to read.
  std::string
    filename = core::str::slashed( core::env::MobiusTestData() )
             + filename_points_001;

  // Load points.
  ptr<pcloud> pts = new pcloud;
  //
  if ( !pts->Load(filename) )
  {
    cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Cannot load points from file %1."
                                                          << filename);
    return res.failure();
  }

  ptr<plane> pln;

  // Build average plane.
  geom_BuildAveragePlane planeAlgo(cf->ProgressNotifier);
  //
  if ( !planeAlgo.Build(pts, pln) )
  {
    cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Cannot build average plane.");
    return res.failure();
  }

  // Verify the result.
  const xyz& O  = pln->GetOrigin();
  const xyz& Du = pln->GetD1();
  const xyz& Dv = pln->GetD2();
  //
  if ( ( O - xyz(2.227936853640657, -6.8177788487915576, -16.700410247411867) ).Modulus() > core_Precision::Resolution3D() )
  {
    cf->ProgressNotifier.SendLogMessage( MobiusErr(Normal) << "Origin of average plane (%1, %2, %3) is not "
                                                              "equal to the expected point."
                                                           << O.X() << O.Y() << O.Z() );
    return res.failure();
  }
  //
  if ( ( Du - xyz(0.14808864587487339, 0.9588368801111562, 0.24228411070817429) ).Modulus() > core_Precision::Resolution3D() )
  {
    cf->ProgressNotifier.SendLogMessage( MobiusErr(Normal) << "Du of average plane (%1, %2, %3) is not "
                                                              "equal to the expected vector."
                                                           << Du.X() << Du.Y() << Du.Z() );
    return res.failure();
  }
  //
  if ( ( Dv - xyz(-0.98763330976881147, 0.13062879033747227, 0.08669812322116055) ).Modulus() > core_Precision::Resolution3D() )
  {
    cf->ProgressNotifier.SendLogMessage( MobiusErr(Normal) << "Dv of average plane (%1, %2, %3) is not "
                                                              "equal to the expected vector."
                                                           << Dv.X() << Dv.Y() << Dv.Z() );
    return res.failure();
  }

  return res.success();
}
