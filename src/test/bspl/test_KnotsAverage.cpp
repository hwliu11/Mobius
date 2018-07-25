//-----------------------------------------------------------------------------
// Created on: 04 November 2013
//-----------------------------------------------------------------------------
// Copyright (c) 2017, Sergey Slyadnev
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
#include <mobius/test_KnotsAverage.h>

// bspl includes
#include <mobius/bspl_KnotsAverage.h>

//-----------------------------------------------------------------------------

//! Test scenario 001.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_KnotsAverage::testCase1_noDerivativeConstraints(const int test_NotUsed(funcID))
{
  outcome res;

  /* --------------------
   *  Prepare input data
   * -------------------- */

  const int n = 5; // Index of the last point
  const int p = 3; // Degree

  const int m = bspl::NumberOfKnots(n, p) - 1;
  if ( m != 9 )
    return res.failure();

  const double t[] = {0.0, 0.1, 0.25, 0.5, 0.75, 1.0};

  /* -----------------
   *  Run calculation
   * ----------------- */

  double* U = new double[m + 1];
  memset(U, 0, (m + 1)*sizeof(double));

  if ( bspl_KnotsAverage::Calculate(t, n, p, m,
                                    bspl_KnotsAverage::Recognize(false, false, false, false),
                                    U) != bspl_KnotsAverage::ErrCode_NoError )
  {
    delete [] U;
    return res.failure();
  }

  /* -------------------
   *  Verify parameters
   * ------------------- */

  // Referential parameters
  double U_ref[] = {0.0, 0.0, 0.0, 0.0, 0.28333333, 0.5, 1.0, 1.0, 1.0, 1.0};

  // Tolerance
  const double tol = 1.0e-6;

  // Compare with tolerance
  for ( int k = 0; k < sizeof(U_ref)/sizeof(double); ++k )
    if ( fabs(U_ref[k] - U[k]) > tol )
      return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 002.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_KnotsAverage::testCase1_endDerivativeConstraints(const int test_NotUsed(funcID))
{
  outcome res;

  /* --------------------
   *  Prepare input data
   * -------------------- */

  const int n = 5; // Index of the last point
  const int p = 3; // Degree

  const int m = (bspl::NumberOfKnots(n, p) - 1) + 2; // Two additional knots for constraints
  if ( m != 11 )
    return res.failure();

  const double t[] = {0.0, 0.1, 0.25, 0.5, 0.75, 1.0};

  /* -----------------
   *  Run calculation
   * ----------------- */

  double* U = new double[m + 1];
  memset(U, 0, (m + 1)*sizeof(double));

  if ( bspl_KnotsAverage::Calculate(t, n, p, m,
                                    bspl_KnotsAverage::Recognize(true, true, false, false),
                                    U) != bspl_KnotsAverage::ErrCode_NoError )
  {
    delete [] U;
    return res.failure();
  }

  /* -------------------
   *  Verify parameters
   * ------------------- */

  // Referential parameters
  double U_ref[] = {0.0, 0.0, 0.0, 0.0, 0.11666666, 0.2833333, 0.5, 0.75, 1.0, 1.0, 1.0, 1.0};

  // Tolerance
  const double tol = 1.0e-6;

  // Compare with tolerance
  for ( int k = 0; k < sizeof(U_ref)/sizeof(double); ++k )
    if ( fabs(U_ref[k] - U[k]) > tol )
      return res.failure();

  return res.success();
}
