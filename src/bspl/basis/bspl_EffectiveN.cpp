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
#include <mobius/bspl_EffectiveN.h>

//! Evaluates non-vanishing basis functions.
//! \param u [in]  parameter value to evaluate basis functions in.
//! \param U [in]  knot vector.
//! \param p [in]  degree of the basis functions to evaluate.
//! \param i [in]  span index the parameter u falls to.
//! \param N [out] evaluated functions. Note that the invoker code
//!                must allocate memory for this array. Its dimension is (p + 1).
void mobius::bspl_EffectiveN::operator()(const double               u,
                                         const std::vector<double>& U,
                                         const int                  p,
                                         const int                  i,
                                         double*                    N) const
{
  // Initial basis function to start iterative evaluation
  N[0] = 1.0;

  // According to computational theory for effective B-spline evaluation,
  // we need to allocate p items for left[] and right[] terms. We will never
  // need left[0] and right[0] terms as these terms vanish due to the
  // organization of calculation scheme. Nevertheless, we prefer to allocate
  // memory for these item as well, just because it is more convenient and
  // more consistent with theory to start indexation from 1
  double* left  = new double[p+1];
  double* right = new double[p+1];
  //
  for ( int k = 0; k < p + 1; ++k ) // Notice that we do not use memset() for
  {                                 // nullification as it will corrupt
    left[k]  = 0;                   // active variables (for ADOL-C)
    right[k] = 0;
  }

  // We start from degree equal to 1. This is because basis functions
  // are constant and equal to 1 for degree = 0. We elevate degree re-using
  // previously calculated B-spline values according to the recurrent
  // formulation given in theory
  for ( int deg = 1; deg <= p; ++deg )
  {
    left[deg]  = u - U[i + 1 - deg];
    right[deg] = U[i + deg] - u;

    // This variable contains value reused on adjacent iterations
    // by the number of evaluated functions
    double savedTerm = 0.0;

    // Now we iterate over the number of evaluated functions. Notice that
    // even though we have (deg + 1) of such functions, we iterate only deg
    // times as the last function can be simply evaluated thanks to savedTerm
    // variable without additional efforts (see theory)
    for ( int idx = 0; idx < deg; ++idx )
    {
      double rightCoeff = N[idx] / (right[idx+1] + left[deg-idx]);
      //
      N[idx]    = savedTerm + right[idx+1]*rightCoeff;
      savedTerm = left[deg-idx]*rightCoeff;
    }
    N[deg] = savedTerm;
  }

  // Release resources
  delete[] left;
  delete[] right;
}
