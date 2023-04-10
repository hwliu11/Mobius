//-----------------------------------------------------------------------------
// Created on: 16 June 2019
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
#include <mobius/test_KnotsUniform.h>

// bspl includes
#include <mobius/bspl_KnotsUniform.h>

//-----------------------------------------------------------------------------

//! Test scenario 01.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_KnotsUniform::test01(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  /* ====================
   *  Prepare input data
   * ==================== */

  const int n = 5; // Index of the last pole.
  const int p = 3; // Degree

  // Reference knot vector.
  std::vector<double> U_ref = {0., 0., 0., 0., 0.3333333, 0.6666666, 1., 1., 1., 1.};

  /* ================================
   *  Compute knot vector and verify
   * ================================ */

  int m = 0;
  std::vector<double> U;
  //
  if ( bspl_KnotsUniform::Calculate(n, p,
                                    m, U) != bspl_KnotsUniform::ErrCode_NoError )
  {
    return res.failure();
  }

  // Tolerance for floating-point values comparison.
  const double tol = 1.0e-6;

  // Compare with tolerance.
  for ( size_t k = 0; k < U_ref.size(); ++k )
    if ( fabs(U_ref[k] - U[k]) > tol )
      return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 02.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_KnotsUniform::test02(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  /* ====================
   *  Prepare input data
   * ==================== */

  const int n = 5; // Index of the last pole.
  const int p = 4; // Degree

  // Reference knot vector.
  std::vector<double> U_ref = {0., 0., 0., 0., 0., 0.5, 1., 1., 1., 1., 1.};

  /* ================================
   *  Compute knot vector and verify
   * ================================ */

  int m = 0;
  std::vector<double> U;
  //
  if ( bspl_KnotsUniform::Calculate(n, p,
                                    m, U) != bspl_KnotsUniform::ErrCode_NoError )
  {
    return res.failure();
  }

  // Tolerance for floating-point values comparison.
  const double tol = 1.0e-6;

  // Compare with tolerance.
  for ( size_t k = 0; k < U_ref.size(); ++k )
    if ( fabs(U_ref[k] - U[k]) > tol )
      return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 03.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_KnotsUniform::test03(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  /* ====================
   *  Prepare input data
   * ==================== */

  const int n = 2; // Index of the last pole.
  const int p = 4; // Degree

  /* ============================================
   *  Compute knot vector: error code is checked
   * ============================================ */

  int m = 0;
  std::vector<double> U;
  //
  if ( bspl_KnotsUniform::Calculate(n, p,
                                    m, U) != bspl_KnotsUniform::ErrCode_ErrNumPoles )
  {
    return res.failure();
  }

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 04.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_KnotsUniform::test04(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  /* ====================
   *  Prepare input data
   * ==================== */

  const int n = 3; // Index of the last pole.
  const int p = 3; // Degree

  // Reference knot vector.
  std::vector<double> U_ref = {0., 0., 0., 0., 1., 1., 1., 1.};

  /* ============================================
   *  Compute knot vector: error code is checked
   * ============================================ */

  int m = 0;
  std::vector<double> U;
  //
  if ( bspl_KnotsUniform::Calculate(n, p,
                                    m, U) != bspl_KnotsUniform::ErrCode_NoError )
  {
    return res.failure();
  }

  // Tolerance for floating-point values comparison.
  const double tol = 1.0e-6;

  // Compare with tolerance.
  for ( size_t k = 0; k < U_ref.size(); ++k )
    if ( fabs(U_ref[k] - U[k]) > tol )
      return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 05.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_KnotsUniform::test05(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  /* ====================
   *  Prepare input data
   * ==================== */

  const int n = 2; // Index of the last pole.
  const int p = 3; // Degree

  /* ============================================
   *  Compute knot vector: error code is checked
   * ============================================ */

  int m = 0;
  std::vector<double> U;
  //
  if ( bspl_KnotsUniform::Calculate(n, p,
                                    m, U) != bspl_KnotsUniform::ErrCode_ErrNumPoles )
  {
    return res.failure();
  }

  return res.success();
}
