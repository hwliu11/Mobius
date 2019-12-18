//-----------------------------------------------------------------------------
// Created on: 10 December 2019
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
#include <mobius/test_SVO.h>

// Test includes
#include <mobius/test_CommonFacilities.h>

// Poly includes
#include <mobius/poly_SVO.h>

//-----------------------------------------------------------------------------

//! Test scenario for verification of corner IDs.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_SVO::testGetCornerID(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  // Access common facilities.
  t_ptr<test_CommonFacilities> cf = test_CommonFacilities::Instance();

  /*  nx | ny | nz | ID
     ----+----+----+----
       0 |  0 |  0 |  0
     ----+----+----+----
       0 |  0 |  1 |  4
     ----+----+----+----
       0 |  1 |  0 |  2
     ----+----+----+----
       0 |  1 |  1 |  6
     ----+----+----+----
       1 |  0 |  0 |  1
     ----+----+----+----
       1 |  0 |  1 |  5
     ----+----+----+----
       1 |  1 |  0 |  3
     ----+----+----+----
       1 |  1 |  1 |  7
     ----+----+----+---- */

  const size_t idFor_000 = poly_SVO::GetCornerID(0, 0, 0);
  const size_t idFor_001 = poly_SVO::GetCornerID(0, 0, 1);
  const size_t idFor_010 = poly_SVO::GetCornerID(0, 1, 0);
  const size_t idFor_011 = poly_SVO::GetCornerID(0, 1, 1);
  const size_t idFor_100 = poly_SVO::GetCornerID(1, 0, 0);
  const size_t idFor_101 = poly_SVO::GetCornerID(1, 0, 1);
  const size_t idFor_110 = poly_SVO::GetCornerID(1, 1, 0);
  const size_t idFor_111 = poly_SVO::GetCornerID(1, 1, 1);

  cf->ProgressNotifier.SendLogMessage(MobiusInfo(Normal) << "SVO corner id for location (0, 0, 0) = %1." << idFor_000);
  cf->ProgressNotifier.SendLogMessage(MobiusInfo(Normal) << "SVO corner id for location (0, 0, 1) = %1." << idFor_001);
  cf->ProgressNotifier.SendLogMessage(MobiusInfo(Normal) << "SVO corner id for location (0, 1, 0) = %1." << idFor_010);
  cf->ProgressNotifier.SendLogMessage(MobiusInfo(Normal) << "SVO corner id for location (0, 1, 1) = %1." << idFor_011);
  cf->ProgressNotifier.SendLogMessage(MobiusInfo(Normal) << "SVO corner id for location (1, 0, 0) = %1." << idFor_100);
  cf->ProgressNotifier.SendLogMessage(MobiusInfo(Normal) << "SVO corner id for location (1, 0, 1) = %1." << idFor_101);
  cf->ProgressNotifier.SendLogMessage(MobiusInfo(Normal) << "SVO corner id for location (1, 1, 0) = %1." << idFor_110);
  cf->ProgressNotifier.SendLogMessage(MobiusInfo(Normal) << "SVO corner id for location (1, 1, 1) = %1." << idFor_111);

  if ( idFor_000 != 0 )
    return res.failure();

  if ( idFor_001 != 4 )
    return res.failure();

  if ( idFor_010 != 2 )
    return res.failure();

  if ( idFor_011 != 6 )
    return res.failure();

  if ( idFor_100 != 1 )
    return res.failure();

  if ( idFor_101 != 5 )
    return res.failure();

  if ( idFor_110 != 3 )
    return res.failure();

  if ( idFor_111 != 7 )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario for verification of corner locations obtained by their IDs.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_SVO::testGetCornerLocation(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  // Access common facilities.
  t_ptr<test_CommonFacilities> cf = test_CommonFacilities::Instance();

  const size_t locations[8][3] =
  {
    {0, 0, 0},
    {1, 0, 0},
    {0, 1, 0},
    {1, 1, 0},
    {0, 0, 1},
    {1, 0, 1},
    {0, 1, 1},
    {1, 1, 1}
  };

  for ( size_t id = 0; id < 8; ++id )
  {
    size_t nx, ny, nz;
    //
    poly_SVO::GetCornerLocation(id, nx, ny, nz);

    cf->ProgressNotifier.SendLogMessage(MobiusInfo(Normal) << "SVO location for id %1 = (%2, %3, %4)."
                                                           << id
                                                           << nx << ny << nz);

    // Verify.
    if ( nx != locations[id][0] || ny != locations[id][1] || nz != locations[id][2] )
      return res.failure();
  }

  return res.success();
}
