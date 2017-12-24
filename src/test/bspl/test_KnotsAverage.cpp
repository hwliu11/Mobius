//-----------------------------------------------------------------------------
// Created on: 04 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

// Own include
#include <mobius/test_KnotsAverage.h>

// bspl includes
#include <mobius/bspl_KnotsAverage.h>

//! Test scenario 001.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
bool mobius::test_KnotsAverage::testCase1_noDerivativeConstraints(const int test_NotUsed(funcID))
{
  /* --------------------
   *  Prepare input data
   * -------------------- */

  const int n = 5; // Index of the last point
  const int p = 3; // Degree

  const int m = bspl::NumberOfKnots(n, p) - 1;
  if ( m != 9 )
    return false;

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
    return false;
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
      return false;

  return true;
}

//! Test scenario 002.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
bool mobius::test_KnotsAverage::testCase1_endDerivativeConstraints(const int test_NotUsed(funcID))
{
  /* --------------------
   *  Prepare input data
   * -------------------- */

  const int n = 5; // Index of the last point
  const int p = 3; // Degree

  const int m = (bspl::NumberOfKnots(n, p) - 1) + 2; // Two additional knots for constraints
  if ( m != 11 )
    return false;

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
    return false;
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
      return false;

  return true;
}
