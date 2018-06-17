//-----------------------------------------------------------------------------
// Created on: 01 August 2013
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
#include <mobius/test_EffectiveN.h>

// bspl includes
#include <mobius/bspl_EffectiveN.h>
#include <mobius/bspl_FindSpan.h>

//-----------------------------------------------------------------------------

//! Test scenario 001: evaluate B-spline in its domain.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
bool mobius::test_EffectiveN::evalInDomain(const int funcID)
{
  const std::vector<double> U = {0.0, 0.0, 0.0, 1, 2, 3, 4, 4, 5, 5, 5};
  const int p = 2;

  bspl_EffectiveN Eval;

  double u = 2.5;
  bspl_FindSpan FindSpan(U, p);
  const int i = FindSpan(u);

  double* N = new double[p+1];
  Eval(u, U, p, i, N);

  // Set description variables
  SetVarDescr("U", U,      ID(), funcID);
  SetVarDescr("p", p,      ID(), funcID);
  SetVarDescr("u", u,      ID(), funcID);
  SetVarDescr("i", i,      ID(), funcID);
  SetVarDescr("N", N, p+1, ID(), funcID);

  if ( N[0] != 0.125 || N[1] != 0.75 || N[2] != 0.125 )
  {
    delete[] N;
    return false;
  }

  delete[] N;
  return true;
}

//-----------------------------------------------------------------------------

//! Test scenario 002: evaluate B-spline out of its domain on the right.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
bool mobius::test_EffectiveN::evalOutDomainLeft(const int funcID)
{
  const std::vector<double> U = {0.0, 0.0, 0.0, 1, 2, 3, 4, 4, 5, 5, 5};
  const int p = 2;

  bspl_EffectiveN Eval;

  double u = 5.1;
  bspl_FindSpan FindSpan(U, p);
  const int i = FindSpan(u);

  double* N = new double[p+1];
  Eval(u, U, p, i, N);

  // Set description variables
  SetVarDescr("U", U,      ID(), funcID);
  SetVarDescr("p", p,      ID(), funcID);
  SetVarDescr("u", u,      ID(), funcID);
  SetVarDescr("i", i,      ID(), funcID);
  SetVarDescr("N", N, p+1, ID(), funcID);

  if ( i != 7 )
  {
    delete[] N;
    return false;
  }

  const double eps = 1e-7;
  //
  if ( fabs(N[0] - 0.01) > eps ||
       fabs(N[1] + 0.22) > eps ||
       fabs(N[2] - 1.21) > eps )
  {
    delete[] N;
    return false;
  }

  delete[] N;
  return true;
}

//-----------------------------------------------------------------------------

//! Test scenario 003: evaluate B-spline out of its domain on the left.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
bool mobius::test_EffectiveN::evalOutDomainRight(const int funcID)
{
  const std::vector<double> U = {0.0, 0.0, 0.0, 1, 2, 3, 4, 4, 5, 5, 5};
  const int p = 2;

  bspl_EffectiveN Eval;

  double u = -0.1;
  bspl_FindSpan FindSpan(U, p);
  const int i = FindSpan(u);

  double* N = new double[p+1];
  Eval(u, U, p, i, N);

  // Set description variables
  SetVarDescr("U", U,      ID(), funcID);
  SetVarDescr("p", p,      ID(), funcID);
  SetVarDescr("u", u,      ID(), funcID);
  SetVarDescr("i", i,      ID(), funcID);
  SetVarDescr("N", N, p+1, ID(), funcID);

  if ( i != 2 )
  {
    delete[] N;
    return false;
  }

  const double eps = 1e-7;
  //
  if ( fabs(N[0] - 1.21)  > eps ||
       fabs(N[1] + 0.215) > eps ||
       fabs(N[2] - 0.005) > eps )
  {
    delete[] N;
    return false;
  }

  delete[] N;
  return true;
}
