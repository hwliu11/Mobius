//-----------------------------------------------------------------------------
// Created on: 17 June 2019
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
#include <mobius/test_ApproxSurf.h>

// Test includes
#include <mobius/test_CommonFacilities.h>

// Geom includes
#include <mobius/geom_ApproxBSurf.h>
#include <mobius/geom_BuildAveragePlane.h>

// Standard includes
#include <fstream>

//-----------------------------------------------------------------------------

// Filenames are specified relatively to MOBIUS_TEST_DATA environment variable.
#define filename_points_001 "points/sampled-surf_01.xyz"

//-----------------------------------------------------------------------------

//! Test scenario 001.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_ApproxSurf::testApprox01(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  // Access common facilities.
  t_ptr<test_CommonFacilities> cf = test_CommonFacilities::Instance();

  // File to read.
  std::string
    filename = core::str::slashed( core::env::MobiusTestData() )
             + filename_points_001;

  // Read point cloud.
  t_ptr<t_pcloud> pts = new t_pcloud;
  //
  if ( !pts->Load(filename) )
    return res.failure();

  // Build average plane.
  t_ptr<t_plane> averagePln;
  //
  geom_BuildAveragePlane planeAlgo;
  //
  if ( !planeAlgo.Build(pts, averagePln) )
  {
    cf->ProgressNotifier.SendLogMessage(MobiusErr(Normal) << "Cannot build average plane.");
    return res.failure();
  }

  return res.success();
}
