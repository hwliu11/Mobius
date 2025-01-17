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

// Own include
#include <mobius/test_ParamsChordLength.h>

// bspl includes
#include <mobius/bspl_ParamsChordLength.h>

//-----------------------------------------------------------------------------

//! Test scenario 001.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_ParamsChordLength::test1(const int funcID)
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
  if ( bspl_ParamsChordLength::Calculate(Q_vec, t) != bspl_ParamsChordLength::ErrCode_NoError )
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
  mobius::test_ParamsChordLength::test2(const int funcID)
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
  if ( bspl_ParamsChordLength::Calculate(Q_vec, t) != bspl_ParamsChordLength::ErrCode_NoError )
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
  double t_ref[4] = {0.0, 0.25, 0.5, 1.0};

  // Tolerance
  const double tol = 1.0e-6;

  // Compare with tolerance
  for ( int k = 0; k < 4; ++k )
    if ( fabs(t_ref[k] - t[k]) > tol )
      return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 003.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_ParamsChordLength::test3(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  /* ----------------------
   *  Prepare input points
   * ---------------------- */

  t_xyz Q[2][2] = { { t_xyz(0.0, 0.0, 0.0), t_xyz(0.0, 1.0, 0.0) },
                    { t_xyz(1.0, 0.0, 0.0), t_xyz(1.0, 1.0, 0.0) } };

  std::vector< std::vector<t_xyz> > Q_vec;
  for ( int k = 0; k < 2; ++k )
  {
    Q_vec.push_back( std::vector<t_xyz>() );
    for ( int l = 0; l < 2; ++l )
      Q_vec[k].push_back( Q[k][l] );
  }

  /* -----------------
   *  Run calculation
   * ----------------- */

  // Collection of parameters
  double u[2] = {0.0, 0.0};
  double v[2] = {0.0, 0.0};

  // Perform
  if ( bspl_ParamsChordLength::Calculate(Q_vec, u, v) != bspl_ParamsChordLength::ErrCode_NoError )
    return res.failure();

  /* -----------------------
   *  Description variables
   * ----------------------- */

  SetVarDescr("n",        1,                           ID(), 003);
  SetVarDescr("m",        1,                           ID(), 003);
  SetVarDescr("params_U", u, sizeof(u)/sizeof(double), ID(), 003);
  SetVarDescr("params_V", v, sizeof(v)/sizeof(double), ID(), 003);

  /* -------------------
   *  Verify parameters
   * ------------------- */

  // Referential parameters
  double u_ref[2] = {0.0, 1.0};
  double v_ref[2] = {0.0, 1.0};

  // Tolerance
  const double tol = 1.0e-6;

  // Compare with tolerance
  for ( int k = 0; k < 2; ++k )
  {
    if ( fabs(u_ref[k] - u[k]) > tol )
      return res.failure();

    if ( fabs(v_ref[k] - v[k]) > tol )
      return res.failure();
  }

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 004.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_ParamsChordLength::test4(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  /* ----------------------
   *  Prepare input points
   * ---------------------- */

  t_xyz Q[3][2] = { { t_xyz(0.0,  0.0, 0.0), t_xyz(0.0,  1.0, 0.0) },
                    { t_xyz(0.75, 0.0, 0.0), t_xyz(0.75, 1.0, 0.0) },
                    { t_xyz(1.0,  0.0, 0.0), t_xyz(1.0,  1.5, 0.0) } };

  std::vector< std::vector<t_xyz> > Q_vec;
  for ( int k = 0; k < 3; ++k )
  {
    Q_vec.push_back( std::vector<t_xyz>() );
    for ( int l = 0; l < 2; ++l )
      Q_vec[k].push_back( Q[k][l] );
  }

  /* -----------------
   *  Run calculation
   * ----------------- */

  // Collection of parameters
  double u[3] = {0.0, 0.0, 0.0};
  double v[2] = {0.0, 0.0};

  // Perform
  if ( bspl_ParamsChordLength::Calculate(Q_vec, u, v) != bspl_ParamsChordLength::ErrCode_NoError )
    return res.failure();

  /* -----------------------
   *  Description variables
   * ----------------------- */

  SetVarDescr("n",        2,                           ID(), 004);
  SetVarDescr("m",        1,                           ID(), 004);
  SetVarDescr("params_U", u, sizeof(u)/sizeof(double), ID(), 004);
  SetVarDescr("params_V", v, sizeof(v)/sizeof(double), ID(), 004);

  /* -------------------
   *  Verify parameters
   * ------------------- */

  // Referential parameters
  double u_ref[3] = {0.0, 0.661475, 1.0};
  double v_ref[2] = {0.0, 1.0};

  // Tolerance
  const double tol = 1.0e-6;

  // Compare with tolerance (U)
  for ( int k = 0; k < 3; ++k )
  {
    if ( fabs(u_ref[k] - u[k]) > tol )
      return res.failure();
  }

  // Compare with tolerance (V)
  for ( int k = 0; k < 2; ++k )
  {
    if ( fabs(v_ref[k] - v[k]) > tol )
      return res.failure();
  }

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 005.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_ParamsChordLength::test5(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  /* ----------------------
   *  Prepare input points
   * ---------------------- */

  t_xyz Q[3][3] = { { t_xyz(0.0,  0.0, 0.0), t_xyz(0.0,  1.0, 0.0), t_xyz(0.0,  1.5,  0.0) },
                    { t_xyz(0.75, 0.0, 0.0), t_xyz(0.75, 1.0, 0.0), t_xyz(0.75, 1.75, 0.0) },
                    { t_xyz(1.0,  0.0, 0.0), t_xyz(1.0,  1.5, 0.0), t_xyz(1.5,  1.75, 0.0) } };

  std::vector< std::vector<t_xyz> > Q_vec;
  for ( int k = 0; k < 3; ++k )
  {
    Q_vec.push_back( std::vector<t_xyz>() );
    for ( int l = 0; l < 3; ++l )
      Q_vec[k].push_back( Q[k][l] );
  }

  /* -----------------
   *  Run calculation
   * ----------------- */

  // Collection of parameters
  double u[3] = {0.0, 0.0, 0.0};
  double v[3] = {0.0, 0.0, 0.0};

  // Perform
  if ( bspl_ParamsChordLength::Calculate(Q_vec, u, v) != bspl_ParamsChordLength::ErrCode_NoError )
    return res.failure();

  /* -----------------------
   *  Description variables
   * ----------------------- */

  SetVarDescr("n",        2,                           ID(), 005);
  SetVarDescr("m",        2,                           ID(), 005);
  SetVarDescr("params_U", u, sizeof(u)/sizeof(double), ID(), 005);
  SetVarDescr("params_V", v, sizeof(v)/sizeof(double), ID(), 005);

  /* -------------------
   *  Verify parameters
   * ------------------- */

  // Referential parameters
  double u_ref[3] = {0.0, 0.612039, 1.0};
  double v_ref[3] = {0.0, 0.655533, 1.0};

  // Tolerance
  const double tol = 1.0e-6;

  // Compare with tolerance (U)
  for ( int k = 0; k < 3; ++k )
  {
    if ( fabs(u_ref[k] - u[k]) > tol )
      return res.failure();
  }

  // Compare with tolerance (V)
  for ( int k = 0; k < 3; ++k )
  {
    if ( fabs(v_ref[k] - v[k]) > tol )
      return res.failure();
  }

  return res.success();
}
