//-----------------------------------------------------------------------------
// Created on: 02 November 2013
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

#ifndef bspl_KnotsAverage_HeaderFile
#define bspl_KnotsAverage_HeaderFile

// bspl includes
#include <mobius/bspl.h>

namespace mobius {

//! \ingroup MOBIUS_BSPL
//!
//! Chooses interpolant's knots according to averaging rule:
//!
//! u_0 = ... = u_p = 0;
//! ...
//! u_j+p = 1/p * Sum(t_i : i=j,(j+p-1));
//! ...
//! u_m-p = ... u_m = 1;
//!
//! Here {t_k} are parameters of the target curve in the data points;
//! p is the degree of B-spline basis functions.
class bspl_KnotsAverage
{
public:

  //! Error codes.
  enum ErrCode
  {
    ErrCode_NoError = 0,
    ErrCode_CaseUnhandled
  };

  //! Possible averaging cases.
  enum Case
  {
    Case_Unhandled = -1,
    Case_PointsOnly,     //!< Knots are averaged for case when there are only points to interpolate.
    Case_Points_D1,      //!< Points and D1 constraints: two additional knots need to be produced.
    Case_Points_D1_D2    //!< Points, D1 and D2 constraints: four additional knots need to be produced.
  };

public:

  mobiusBSpl_EXPORT static Case
    Recognize(const bool D1_start,
              const bool D1_end,
              const bool D2_start,
              const bool D2_end);

public:

  //! Calculates knot vector by averaging technique.
  //! \param t              [in]  curve's parameters.
  //! \param n              [in]  number of curve's parameters the knots will be based on.
  //! \param p              [in]  B-spline basis degree.
  //! \param m              [in]  last knot index.
  //! \param averaging_case [in]  averaging context.
  //! \param U              [out] output knots.
  //! \return error code.
  static ErrCode Calculate(const adouble* t,
                           const int     n,
                           const int     p,
                           const int     m,
                           const Case    averaging_case,
                           adouble*       U)
  {
    // TODO: test this algo
    // TODO: provide consistency checks via error codes

    if ( averaging_case == Case_Unhandled )
      return ErrCode_CaseUnhandled; // Bad composition of constraints

    for ( int k = 0; k <= p; ++k )
      U[k] = 0.0;

    for ( int k = m - p; k <= m; ++k )
      U[k] = 1.0;

    // First bound for j index used for averaging
    int j_start = (averaging_case == Case_PointsOnly) ? 1 : 0;

    // Second bound for j index used for averaging
    int j_end;
    if ( averaging_case == Case_PointsOnly )
      j_end = n - p;
    else // Points_D1 || Points_D1_D2
      j_end = n - p + 1;

    // Do averaging
    for ( int j = j_start; j <= j_end; ++j )
    {
      adouble avgSum = 0.0;
      const int i_start = j;
      const int i_end   = j + p - 1;
      //
      for ( int i = i_start; i <= i_end; ++i )
        avgSum += t[i];

      int U_shift;
      if ( averaging_case == Case_PointsOnly )
        U_shift = 0;
      else if ( averaging_case == Case_Points_D1 )
        U_shift = 1;
      else // Case_Points_D1_D2
        U_shift = 2;

      U[p + j + U_shift] = avgSum / p;
    }

    // If there are 4 constraints, we need to complete knot vector manually
    if ( averaging_case == Case_Points_D1_D2 )
    {
      U[p + 1] = U[p + 2] / 2;
      U[n + 4] = (1.0 + U[n + 3]) / 2;
    }

    return ErrCode_NoError;
  }

private:

  bspl_KnotsAverage() {}
  bspl_KnotsAverage(const bspl_KnotsAverage&) {}

};

};

#endif
