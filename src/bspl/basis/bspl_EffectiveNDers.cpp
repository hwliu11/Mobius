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
#include <mobius/bspl_EffectiveNDers.h>

// core includes
#include <mobius/core_HeapAlloc.h>

//! Evaluates non-vanishing basis functions with their derivatives of
//! (k)-th order.
//! \param u      [in]  parameter value to evaluate functions in.
//! \param U      [in]  knot vector.
//! \param p      [in]  desired degree of basis functions to evaluate.
//! \param span_i [in]  span index the parameter u falls to.
//! \param order  [in]  maximal order order derivatives to calculate.
//! \param ders   [out] evaluated functions. Note that the invoker code
//!                     must allocate memory for this array. Its dimension is
//!                     (p + 1)*(k + 1).
void mobius::bspl_EffectiveNDers::operator()(const adouble               u,
                                             const std::vector<adouble>& U,
                                             const int                  p,
                                             const int                  span_i,
                                             const int                  order,
                                             adouble**                   ders) const
{
  ptr<alloc2d> localAlloc;

  // Prepare working arrays:
  //
  // - left[] and right[]: to store knot differences.
  // - ndu[][]:            to store working matrix.
  adouble left[mobiusBSpl_MaxDegree], right[mobiusBSpl_MaxDegree];
  adouble ndu[mobiusBSpl_MaxDegree][mobiusBSpl_MaxDegree];
  //
  memset(left,  0, sizeof(adouble)*mobiusBSpl_MaxDegree);
  memset(right, 0, sizeof(adouble)*mobiusBSpl_MaxDegree);
  memset(ndu,   0, sizeof(adouble)*mobiusBSpl_MaxDegree*mobiusBSpl_MaxDegree);
  /*for ( int i = 0; i < mobiusBSpl_MaxDegree; ++i )
  {
    left[i] = right[i] = 0.0;
    for ( int j = 0; j < mobiusBSpl_MaxDegree; ++j )
      ndu[i][j] = 0.0;
  }*/

  //------------------------------------------------------
  // Evaluate basis functions along with knot differences
  //------------------------------------------------------

  // Initial basis function to start iterative evaluation
  ndu[0][0] = 1.0;

  // We start from degree equal to 1. This is because basis functions
  // are constant and equal to 1 for degree = 0. We elevate degree re-using
  // previously calculated B-spline values according to the recurrent
  // formulation given in theory
  for ( int j = 1; j <= p; ++j )
  {
    left[j]  = u - U[span_i+1-j];
    right[j] = U[span_i+j] - u;

    // This variable contains value reused on adjacent iterations
    // by the number of evaluated functions
    adouble savedTerm = 0.0;

    // Now we iterate over the number of evaluated functions. Notice that
    // even though we have (deg + 1) of such functions, we iterate only deg
    // times as the last function can be simply evaluated thanks to savedTerm
    // variable without additional efforts (see theory)
    for ( int r = 0; r < j; ++r )
    {
      // Knot differences (lower triangle)
      ndu[j][r]   = right[r+1] + left[j-r];
      adouble temp = ndu[r][j-1] / ndu[j][r];

      // Basis functions (upper triangle)
      ndu[r][j] = savedTerm + right[r+1]*temp;
      savedTerm = left[j-r]*temp;
    }
    ndu[j][j] = savedTerm;
  }

  //----------------------
  // Evaluate derivatives
  //----------------------

  // Coefficients.
  adouble** a;
  if ( m_pAlloc.IsNull() )
  {
    localAlloc = new alloc2d;
    a = localAlloc->Allocate(2, order+1, true); // Two working rows
  }
  else
    a = m_pAlloc->Access(m_iMemBlock).Ptr;

  // Load basis functions into the first row of result matrix
  for ( int j = 0; j <= p; ++j )
    ders[0][j] = ndu[j][p]; // Take all effective functions of degree p

  // Loop over the effective indices
  for ( int r = 0; r <= p; ++r )
  {
    int s1 = 0, s2 = 1; // Alternate rows in array a
    a[0][0] = 1.0;

    // Loop to compute (k)-th derivative
    for ( int k = 1; k <= order; ++k )
    {
      adouble d = 0.0; // Derivative being evaluated
      int   rk = r-k, pk = p-k;

      if ( r >= k )
      {
        a[s2][0] = a[s1][0]/ndu[pk+1][rk];
        d        = a[s2][0]*ndu[rk][pk];
      }

      int j1, j2;
      if ( rk >= -1 )
        j1 = 1;
      else
        j1 = -rk;
      //
      if ( r-1 <= pk )
        j2 = k-1;
      else
        j2 = p-r;

      for ( int j = j1; j <= j2; ++j )
      {
        a[s2][j] =  (a[s1][j]-a[s1][j-1])/ndu[pk+1][rk+j];
        d        += a[s2][j]*ndu[rk+j][pk];
      }

      if ( r <= pk )
      {
        a[s2][k] = -a[s1][k-1]/ndu[pk+1][r];
        d        += a[s2][k]*ndu[r][pk];
      }

      ders[k][r] = d;
      int j = s1; s1 = s2; s2 = j; // Switch rows
    }
  }

  // Multiply through by the correct factors
  int r = p;
  for ( int k = 1; k <= order; ++k )
  {
    for ( int j = 0; j <= p; ++j )
      ders[k][j] *= r;

    r *= (p-k);
  }
}
