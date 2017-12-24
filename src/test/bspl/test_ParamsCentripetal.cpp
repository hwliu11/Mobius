//-----------------------------------------------------------------------------
// Created on: 16 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

// Own include
#include <mobius/test_ParamsCentripetal.h>

// bspl includes
#include <mobius/bspl_ParamsCentripetal.h>

//! Test scenario 001.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
bool mobius::test_ParamsCentripetal::test1(const int test_NotUsed(funcID))
{
  /* ----------------------
   *  Prepare input points
   * ---------------------- */

  xyz Q[3] = { xyz(0.0, 0.0, 0.0),
               xyz(1.0, 0.0, 0.0),
               xyz(2.0, 0.0, 0.0) };

  std::vector<xyz> Q_vec;
  for ( int k = 0; k < sizeof(Q)/sizeof(xyz); ++k )
    Q_vec.push_back(Q[k]);

  /* -----------------
   *  Run calculation
   * ----------------- */

  // Collection of parameters
  double t[3] = {0.0, 0.0, 0.0};

  // Perform
  if ( bspl_ParamsCentripetal::Calculate(Q_vec, t) != bspl_ParamsCentripetal::ErrCode_NoError )
    return false;

  /* -----------------------
   *  Description variables
   * ----------------------- */

  SetVarDescr("n", 2, ID(), 1);
  SetVarDescr("params", t, 3, ID(), 1);

  /* -------------------
   *  Verify parameters
   * ------------------- */

  // Referential parameters
  double t_ref[3] = {0.0, 0.5, 1.0};

  // Tolerance
  const double tol = 1.0e-6;

  // Compare with tolerance
  for ( int k = 0; k < 3; ++k )
    if ( fabs(t_ref[k] - t[k]) > tol )
      return false;

  return true;
}

//! Test scenario 002.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
bool mobius::test_ParamsCentripetal::test2(const int test_NotUsed(funcID))
{
  /* ----------------------
   *  Prepare input points
   * ---------------------- */

  xyz Q[4] = { xyz(0.0, 0.0, 0.0),
               xyz(1.0, 0.0, 0.0),
               xyz(2.0, 0.0, 0.0),
               xyz(4.0, 0.0, 0.0)};

  std::vector<xyz> Q_vec;
  for ( int k = 0; k < sizeof(Q)/sizeof(xyz); ++k )
    Q_vec.push_back(Q[k]);

  /* -----------------
   *  Run calculation
   * ----------------- */

  // Collection of parameters
  double t[4] = {0.0, 0.0, 0.0, 0.0};

  // Perform
  if ( bspl_ParamsCentripetal::Calculate(Q_vec, t) != bspl_ParamsCentripetal::ErrCode_NoError )
    return false;

  /* -----------------------
   *  Description variables
   * ----------------------- */

  SetVarDescr("n", 3, ID(), 2);
  SetVarDescr("params", t, 4, ID(), 2);

  /* -------------------
   *  Verify parameters
   * ------------------- */

  // Referential parameters
  const double d = 2.0 + sqrt(2.0);
  double t_ref[4] = {0.0, 1.0/d, 2.0/d, 1.0};

  // Tolerance
  const double tol = 1.0e-6;

  // Compare with tolerance
  for ( int k = 0; k < 4; ++k )
    if ( fabs(t_ref[k] - t[k]) > tol )
      return false;

  return true;
}
