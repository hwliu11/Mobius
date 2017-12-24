//-----------------------------------------------------------------------------
// Created on: 10 February 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

// Own include
#include <mobius/test_EffectiveNDers.h>

// core includes
#include <mobius/core_HeapAlloc.h>

// bspl includes
#include <mobius/bspl_EffectiveNDers.h>
#include <mobius/bspl_FindSpan.h>

//! Test scenario 001.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
bool mobius::test_EffectiveNDers::test1(const int funcID)
{
  const std::vector<double> U = {0, 0, 0, 1, 2, 3, 4, 4, 5, 5, 5};
  const int p = 2;
  const int n = 3;

  bspl_EffectiveNDers Eval;

  double u = 2.5;
  bspl_FindSpan FindSpan(U, p);
  const int span_i = FindSpan(u);

  // Prepare result matrix
  core_HeapAlloc2D<double> Alloc;
  double** dN = Alloc.Allocate(n+1, p+1, false);

  // Evaluate
  Eval(u, U, p, span_i, n, dN);

  // Set description variables
  SetVarDescr("U",  U,            ID(), funcID);
  SetVarDescr("p",  p,            ID(), funcID);
  SetVarDescr("u",  u,            ID(), funcID);
  SetVarDescr("i",  span_i,       ID(), funcID);
  SetVarDescr("n",  n,            ID(), funcID);
  SetVarDescr("dN", dN, n+1, p+1, ID(), funcID);

  // Referential output
  double dN_ref[][3] =
    { { 0.125,  0.750, 0.125},   // (k) = 0
      {-0.500,  0.000, 0.500},   // (k) = 1
      { 1.000, -2.000, 1.000},   // (k) = 2
      { 0.000,  0.000, 0.000} }; // (k) = 3

  // Validate results
  bool isSuccess = true;
  for ( int i = 0; i < n+1; ++i )
    for ( int j = 0; j < p+1; ++j )
      if ( dN[i][j] != dN_ref[i][j] )
      {
        isSuccess = false;
        break;
      }

  return isSuccess;
}

//! Test scenario 002.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
bool mobius::test_EffectiveNDers::test2(const int funcID)
{
  const std::vector<double> U = {0, 0, 0, 0, 1, 2, 3, 4, 4, 5, 5, 5, 5};
  const int p = 3;
  const int n = 4;

  bspl_EffectiveNDers Eval;

  double u = 3.5;
  bspl_FindSpan FindSpan(U, p);
  const int span_i = FindSpan(u);

  // Prepare result matrix
  core_HeapAlloc2D<double> Alloc;
  double** dN = Alloc.Allocate(n+1, p+1, false);

  // Evaluate
  Eval(u, U, p, span_i, n, dN);

  // Set description variables
  SetVarDescr("U",  U,            ID(), funcID);
  SetVarDescr("p",  p,            ID(), funcID);
  SetVarDescr("u",  u,            ID(), funcID);
  SetVarDescr("i",  span_i,       ID(), funcID);
  SetVarDescr("n",  n,            ID(), funcID);
  SetVarDescr("dN", dN, n+1, p+1, ID(), funcID);

  // Referential output
  double dN_ref[][4] =
    { { 0.0208333,  0.260417,  0.65625, 0.0625},   // (k) = 0
      {-0.1250000, -0.812500,  0.56250, 0.3750},   // (k) = 1
      { 0.5000000,  0.250000, -2.25000, 1.5000},   // (k) = 2
      {-1.0000000,  5.500000, -7.50000, 3.0000},   // (k) = 3
      { 0.0000000,  0.0000000, 0.00000, 0.0000} }; // (k) = 4

  // Validate results
  const double tol = 1.0e-4; // Compare with tolerance
  bool isSuccess = true;
  for ( int i = 0; i < n+1; ++i )
    for ( int j = 0; j < p+1; ++j )
      if ( fabs(dN[i][j] - dN_ref[i][j]) > tol )
      {
        isSuccess = false;
        break;
      }

  return isSuccess;
}

//! Test scenario 003 (validated visually).
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
bool mobius::test_EffectiveNDers::test3(const int funcID)
{
  const std::vector<double> U = {0.0, 0.0, 0.0, 0.0, 2.0, 4.0, 6.0, 8.0, 8.0, 8.0, 8.0};
  const int p = 3;
  const int n = 1;

  bspl_EffectiveNDers Eval;

  double u = 1.5;
  bspl_FindSpan FindSpan(U, p);
  const int span_i = FindSpan(u);

  // Prepare result matrix
  core_HeapAlloc2D<double> Alloc;
  double** dN = Alloc.Allocate(n+1, p+1, false);

  // Evaluate
  Eval(u, U, p, span_i, n, dN);

  // Set description variables
  SetVarDescr("U",  U,            ID(), funcID);
  SetVarDescr("p",  p,            ID(), funcID);
  SetVarDescr("u",  u,            ID(), funcID);
  SetVarDescr("i",  span_i,       ID(), funcID);
  SetVarDescr("n",  n,            ID(), funcID);
  SetVarDescr("dN", dN, n+1, p+1, ID(), funcID);

  // Referential output
  double dN_ref[][4] =
    { { 0.015625,  0.457031, 0.457031, 0.0703125},   // (k) = 0
      {-0.093750, -0.398438, 0.351563, 0.1406259} }; // (k) = 1

  // Validate results
  const double tol = 1.0e-4; // Compare with tolerance
  bool isSuccess = true;
  for ( int i = 0; i < n+1; ++i )
    for ( int j = 0; j < p+1; ++j )
      if ( fabs(dN[i][j] - dN_ref[i][j]) > tol )
      {
        isSuccess = false;
        break;
      }

  return isSuccess;
}
