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
#include <mobius/test_ParamsCentripetal.h>

// bspl includes
#include <mobius/bspl_ParamsCentripetal.h>

//-----------------------------------------------------------------------------

//! Test scenario 001.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_ParamsCentripetal::test1(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  /* ----------------------
   *  Prepare input points
   * ---------------------- */

  t_xyz Q[3] = { t_xyz(0.0, 0.0, 0.0),
                 t_xyz(1.0, 0.0, 0.0),
                 t_xyz(2.0, 0.0, 0.0) };

  std::vector<t_xyz> Q_vec;
  for ( int k = 0; k < sizeof(Q)/sizeof(t_xyz); ++k )
    Q_vec.push_back(Q[k]);

  /* -----------------
   *  Run calculation
   * ----------------- */

  // Collection of parameters
  double t[3] = {0.0, 0.0, 0.0};

  // Perform
  if ( bspl_ParamsCentripetal::Calculate(Q_vec, t) != bspl_ParamsCentripetal::ErrCode_NoError )
    return res.failure();

  /* -----------------------
   *  Description variables
   * ----------------------- */

  SetVarDescr("n", 2, ID(), 1);
  SetVarDescr("params", t, 3, ID(), 1);

  /* -------------------
   *  Verify parameters
   * ------------------- */

  // Referential parameters
  double t_ref[3] = {0.0, 0.5, 1.0};

  // Tolerance
  const double tol = 1.0e-6;

  // Compare with tolerance
  for ( int k = 0; k < 3; ++k )
    if ( fabs(t_ref[k] - t[k]) > tol )
      return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 002.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_ParamsCentripetal::test2(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  /* ----------------------
   *  Prepare input points
   * ---------------------- */

  t_xyz Q[4] = { t_xyz(0.0, 0.0, 0.0),
                 t_xyz(1.0, 0.0, 0.0),
                 t_xyz(2.0, 0.0, 0.0),
                 t_xyz(4.0, 0.0, 0.0)};

  std::vector<t_xyz> Q_vec;
  for ( int k = 0; k < sizeof(Q)/sizeof(t_xyz); ++k )
    Q_vec.push_back(Q[k]);

  /* -----------------
   *  Run calculation
   * ----------------- */

  // Collection of parameters
  double t[4] = {0.0, 0.0, 0.0, 0.0};

  // Perform
  if ( bspl_ParamsCentripetal::Calculate(Q_vec, t) != bspl_ParamsCentripetal::ErrCode_NoError )
    return res.failure();

  /* -----------------------
   *  Description variables
   * ----------------------- */

  SetVarDescr("n", 3, ID(), 2);
  SetVarDescr("params", t, 4, ID(), 2);

  /* -------------------
   *  Verify parameters
   * ------------------- */

  // Referential parameters
  const double d = 2.0 + sqrt(2.0);
  double t_ref[4] = {0.0, 1.0/d, 2.0/d, 1.0};

  // Tolerance
  const double tol = 1.0e-6;

  // Compare with tolerance
  for ( int k = 0; k < 4; ++k )
    if ( fabs(t_ref[k] - t[k]) > tol )
      return res.failure();

  return res.success();
}
