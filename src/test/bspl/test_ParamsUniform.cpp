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
#include <QrTest_ParamsUniform.h>

// QrBSpl includes
#include <QrBSpl_ParamsUniform.h>

//! Test scenario 001.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
bool QrTest_ParamsUniform::test1(const int QrTest_NotUsed(funcID))
{
  // Index of the last parameter
  const int n = 2;

  // Collection of parameters
  double* t = new double[n + 1];
  memset(t, 0, (n + 1)*sizeof(double));

  // Perform
  if ( QrBSpl_ParamsUniform::Calculate(n, t) != QrBSpl_ParamsUniform::ErrCode_NoError )
  {
    delete[] t;
    return false;
  }

  /* -----------------------
   *  Description variables
   * ----------------------- */

  SetVarDescr("n", n, ID(), 1);
  SetVarDescr("params", t, n + 1, ID(), 1);

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
    {
      delete[] t;
      return false;
    }

  delete[] t;
  return true;
}

//! Test scenario 002.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
bool QrTest_ParamsUniform::test2(const int QrTest_NotUsed(funcID))
{
  // Index of the last parameter
  const int n = 3;

  // Collection of parameters
  double* t = new double[n + 1];
  memset(t, 0, (n + 1)*sizeof(double));

  // Perform
  if ( QrBSpl_ParamsUniform::Calculate(n, t) != QrBSpl_ParamsUniform::ErrCode_NoError )
  {
    delete[] t;
    return false;
  }

  /* -----------------------
   *  Description variables
   * ----------------------- */

  SetVarDescr("n", n, ID(), 2);
  SetVarDescr("params", t, n + 1, ID(), 2);

  /* -------------------
   *  Verify parameters
   * ------------------- */

  // Referential parameters
  double t_ref[4] = {0.0, 0.3333, 0.6666, 1.0};

  // Tolerance
  const double tol = 1.0e-4;

  // Compare with tolerance
  for ( int k = 0; k < 4; ++k )
    if ( fabs(t_ref[k] - t[k]) > tol )
    {
      delete[] t;
      return false;
    }

  delete[] t;
  return true;
}
