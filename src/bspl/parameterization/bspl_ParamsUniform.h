//-----------------------------------------------------------------------------
// Created on: 16 November 2013
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

#ifndef bspl_ParamsUniform_HeaderFile
#define bspl_ParamsUniform_HeaderFile

// bspl includes
#include <mobius/bspl.h>

// core includes
#include <mobius/core_XYZ.h>

namespace mobius {

//! \ingroup MOBIUS_BSPL
//!
//! Chooses target curve's parameters according to the simplest uniform
//! distribution rule:
//!
//! t_0 = 0;
//! ...
//! t_k = k/n;
//! ...
//! t_n = 1;
class bspl_ParamsUniform
{
public:

  //! Error codes.
  enum ErrCode
  {
    ErrCode_NoError = 0,
    ErrCode_CannotProceedWithSolePoint,
  };

public:

  //! Calculates parameter values by uniform distribution rule.
  //! \param n [in]  index of the last parameter in 0-based collection.
  //! \param t [out] calculated parameter values.
  //! \return error code.
  static ErrCode Calculate(const int n,
                           double*   t)
  {
    if ( n < 2 )
      return ErrCode_CannotProceedWithSolePoint; // Cannot proceed with a sole point

    // First parameter
    t[0] = 0.0;

    // Calculate next parameter
    for ( int k = 1; k < n; ++k )
    {
      t[k] = (double) k / n;
    }

    // Last parameter
    t[n] = 1.0;

    // Success
    return ErrCode_NoError;
  }

private:

  bspl_ParamsUniform() = delete;
  bspl_ParamsUniform(const bspl_ParamsUniform&) = delete;

};

}

#endif
