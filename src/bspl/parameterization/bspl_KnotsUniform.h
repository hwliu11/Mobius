//-----------------------------------------------------------------------------
// Created on: 16 June 2019
//-----------------------------------------------------------------------------
// Copyright (c) 2019-present, Sergey Slyadnev
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

#ifndef bspl_KnotsUniform_HeaderFile
#define bspl_KnotsUniform_HeaderFile

// bspl includes
#include <mobius/bspl.h>

namespace mobius {

//! \ingroup MOBIUS_BSPL
//!
//! Chooses knot vector in a uniform way:
//!
//! u_0 = ... = u_p = 0;
//! ...
//! u_j = 1/(m-2p-1) for j = p+1 till j = n;
//! ...
//! u_m-p = ... u_m = 1;
//!
//! Here `(m-2p-1)` is the number of knots between clamps.
class bspl_KnotsUniform
{
public:

  //! Error codes.
  enum ErrCode
  {
    ErrCode_NoError = 0,
    ErrCode_ErrNumPoles, //!< Few number of poles.
    ErrCode_GeneralError //!< Any problem ;)
  };

public:

  //! Calculates knot vector.
  //! \param[in]  n 0-based index of the last pole.
  //! \param[in]  p B-spline function degree.
  //! \param[out] m 0-based index of the last knot.
  //! \param[out] U output knot vector of length `m + 1`.
  //! \return error code.
  static ErrCode Calculate(const int            n,
                           const int            p,
                           int&                 m,
                           std::vector<double>& U)
  {
    if ( p <= 0 || n <= 0 )
      return ErrCode_GeneralError; // Contract check.

    m = bspl::M(n, p);

    if ( m < 2*p + 1 )
      return ErrCode_ErrNumPoles; // Contract check.

    // Allocate knot vector.
    U.resize(m + 1);

    for ( int k = 0; k <= p; ++k )
      U[k] = 0.0;

    for ( int k = m - p; k <= m; ++k )
      U[k] = 1.0;

    // Indices for the knots between clamps.
    const int numDistinct = m - 2*p - 1;
    const int j_start     = p + 1;
    const int j_end       = n;
    //
    if ( numDistinct <= 0 )
      return ErrCode_NoError;

    const double delta = 1.0/(numDistinct + 1);

    // Select distinct knots.
    for ( int j = j_start; j <= j_end; ++j )
    {
      U[j] = U[j-1] + delta;
    }

    return ErrCode_NoError;
  }

private:

  bspl_KnotsUniform() = delete;
  bspl_KnotsUniform(const bspl_KnotsUniform&) = delete;

};

}

#endif
