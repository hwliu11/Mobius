//-----------------------------------------------------------------------------
// Created on: 14 June 2013
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

#ifndef bspl_N_HeaderFile
#define bspl_N_HeaderFile

// bspl includes
#include <mobius/bspl_FindSpan.h>

namespace mobius {

//! \ingroup MOBIUS_BSPL
//!
//! Algorithm calculating B-spline basis function.
//! ATTENTION: this algorithm is not optimal!
//!
//! \todo complete description
class bspl_N
{
public:

  double operator()(const double               u,
                    const std::vector<double>& U,
                    const int                  p,
                    const int                  i) const
  {
    bspl_FindSpan FindSpan(U, p);
    const int nU       = (int) U.size();
    const int span_idx = FindSpan(u);
    const int m        = nU - 1;

    // Recurrent Cox-deBoor formulation start with 1-level step function
    if ( p == 0 )
    {
      double val = ( (span_idx == i) ? 1.0 : 0.0 );
      return val;
    }

    // Special cases
    if ( (i == 0 && u == U[0]) || (i == m-p-1 && u == U[m]) )
    {
      double val = 1.0;
      return val;
    }

    bspl_N N;
    double N_pprev_icurr = N(u, U, p - 1, i);
    double N_pprev_inext = N(u, U, p - 1, i + 1);
    double deltaU1 = (U[i + p] - U[i]);
    double deltaU2 = (U[i + p + 1] - U[i + 1]);

    const double prec = 1.0e-10;
    double N1, N2;

    if ( fabs(N_pprev_icurr) < prec && fabs(deltaU1) < prec )
      N1 = 0.0;
    else
      N1 = N_pprev_icurr * (u - U[i]) / (U[i + p] - U[i]);

    if ( fabs(N_pprev_inext) < prec && fabs(deltaU2) < prec )
      N2 = 0.0;
    else
    {
      const double urange1 = U[i + p + 1] - u;
      const double urange2 = U[i + p + 1] - U[i + 1];
      if ( fabs(urange1) < prec && fabs(urange2) < prec )
        N2 = 0.0;
      else
        N2 = N_pprev_inext * (U[i + p + 1] - u) / (U[i + p + 1] - U[i + 1]);
    }

    return N1 + N2;
  }

};

}

#endif
