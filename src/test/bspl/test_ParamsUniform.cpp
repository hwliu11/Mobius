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

// Own include
#include <mobius/test_ParamsUniform.h>

// bspl includes
#include <mobius/bspl_ParamsUniform.h>

//! Test scenario 001.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_ParamsUniform::test1(const int test_NotUsed(funcID))
{
  outcome res( DescriptionFn() );

  // Index of the last parameter
  const int n = 2;

  // Collection of parameters
  adouble* t = new adouble[n + 1];

#if defined USE_ADOLC
  for ( int k = 0; k < n + 1; ++k )
    t[k] = 0.0;
#else
  memset(t, 0, (n + 1)*sizeof(double));
#endif

  // Perform
  if ( bspl_ParamsUniform::Calculate(n, t) != bspl_ParamsUniform::ErrCode_NoError )
  {
    delete[] t;
    return res.failure();
  }

  /* -----------------------
   *  Description variables
   * ----------------------- */

  SetVarDescr("n", n, ID(), 1);
  SetVarDescr("params", t, n + 1, ID(), 1);

  /* -------------------
   *  Verify parameters
   * ------------------- */

  // Referential parameters
  adouble t_ref[3] = {0.0, 0.5, 1.0};

  // Tolerance
  const adouble tol = 1.0e-6;

  // Compare with tolerance
  for ( int k = 0; k < 3; ++k )
    if ( fabs(t_ref[k] - t[k]) > tol )
    {
      delete[] t;
      return res.failure();
    }

  delete[] t;
  return res.success();
}

//! Test scenario 002.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_ParamsUniform::test2(const int test_NotUsed(funcID))
{
  outcome res( DescriptionFn() );

  // Index of the last parameter
  const int n = 3;

  // Collection of parameters
  adouble* t = new adouble[n + 1];

#if defined USE_ADOLC
  for ( int k = 0; k < n + 1; ++k )
    t[k] = 0.0;
#else
  memset(t, 0, (n + 1)*sizeof(double));
#endif

  // Perform
  if ( bspl_ParamsUniform::Calculate(n, t) != bspl_ParamsUniform::ErrCode_NoError )
  {
    delete[] t;
    return res.failure();
  }

  /* -----------------------
   *  Description variables
   * ----------------------- */

  SetVarDescr("n", n, ID(), 2);
  SetVarDescr("params", t, n + 1, ID(), 2);

  /* -------------------
   *  Verify parameters
   * ------------------- */

  // Referential parameters
  adouble t_ref[4] = {0.0, 0.3333, 0.6666, 1.0};

  // Tolerance
  const adouble tol = 1.0e-4;

  // Compare with tolerance
  for ( int k = 0; k < 4; ++k )
    if ( fabs(t_ref[k] - t[k]) > tol )
    {
      delete[] t;
      return res.failure();
    }

  delete[] t;
  return res.success();
}
