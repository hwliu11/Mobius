//-----------------------------------------------------------------------------
// Created on: 06 March 2015
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
#include <mobius/bspl_RefineKnots.h>

// core includes
#include <mobius/core_HeapAlloc.h>

// bspl includes
#include <mobius/bspl_FindSpan.h>

//! Implements knot insertion algorithm.
//! \param n    [in]  index of the last pole in original curve.
//! \param p    [in]  degree.
//! \param U    [in]  original knot vector.
//! \param Pw   [in]  original poles.
//! \param X    [in]  knots to insert.
//! \param r    [in]  index of the last element in X.
//! \param Ubar [out] resulting knot vector.
//! \param Qw   [out] resulting poles.
//! \return true in case of success, false -- otherwise.
bool mobius::bspl_RefineKnots::operator()(const int                  n,
                                          const int                  p,
                                          const std::vector<adouble>& U,
                                          const std::vector<xyz>&    Pw,
                                          const adouble*              X,
                                          const int                  r,
                                          std::vector<adouble>&       Ubar,
                                          std::vector<xyz>&          Qw) const
{
  const int m  = n + p + 1;
  const int nq = n + r + 1;

  bspl_FindSpan FindSpan(U, p);
  int a = FindSpan(X[0]);
  int b = FindSpan(X[r]);
  b = b + 1;

  for ( int i = 0; i <= nq; ++i )
    Qw.push_back( xyz() );

  for ( int j = 0;     j <= a - p; ++j ) Qw[j]           = Pw[j];
  for ( int j = b - 1; j <= n;     ++j ) Qw[j + r + 1]   = Pw[j];
  for ( int j = 0;     j <= a;     ++j ) Ubar[j]         = U[j];
  for ( int j = b + p; j <= m;     ++j ) Ubar[j + r + 1] = U[j];

  int i = b + p - 1;
  int k = b + p + r;

  for ( int j = r; j >= 0; --j )
  {
    while ( X[j] <= U[i] && i > a )
    {
      Qw[k - p - 1] = Pw[i - p - 1];
      Ubar[k]       = U[i];
      k             = k - 1;
      i             = i - 1;
    }
    Qw[k - p - 1] = Qw[k - p];
    for ( int l = 1; l <= p; ++l )
    {
      const int ind   = k - p + l;
      adouble    alpha = Ubar[k + l] - X[j];

      if ( fabs(alpha) == 0.0 )
        Qw[ind - 1] = Qw[ind];
      else
      {
        alpha       = alpha/(Ubar[k + l] - U[i - p + l]);
        Qw[ind - 1] = Qw[ind - 1]*alpha + Qw[ind]*(1.0 - alpha);
      }
    }
    Ubar[k] = X[j];
    k       = k - 1;
  }

  return true;
}
