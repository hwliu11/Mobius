//-----------------------------------------------------------------------------
// Created on: 10 February 2014
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
#include <mobius/test_EffectiveNDers.h>

// core includes
#include <mobius/core_HeapAlloc.h>

// bspl includes
#include <mobius/bspl_EffectiveNDers.h>
#include <mobius/bspl_FindSpan.h>

//! Test scenario 001.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome mobius::test_EffectiveNDers::test1(const int funcID)
{
  outcome res;

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

  return isSuccess ? res.success() : res.failure();
}

//! Test scenario 002.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome mobius::test_EffectiveNDers::test2(const int funcID)
{
  outcome res;

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

  return isSuccess ? res.success() : res.failure();
}

//! Test scenario 003 (validated visually).
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome mobius::test_EffectiveNDers::test3(const int funcID)
{
  outcome res;

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

  return isSuccess ? res.success() : res.failure();
}
