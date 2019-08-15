//-----------------------------------------------------------------------------
// Created on: 15 January 2014
//-----------------------------------------------------------------------------
// Copyright (c) 2014-present, Sergey Slyadnev
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
#include <mobius/test_Quaternion.h>

// core includes
#include <mobius/core_Quaternion.h>

// Standard includes
#include <math.h>

//-----------------------------------------------------------------------------

mobius::outcome
  mobius::test_Quaternion::testRodrigues(const core_XYZ& axis,
                                         const double    angleRad,
                                         const core_XYZ& lambdaRef,
                                         const int       funcID)
{
  outcome res( DescriptionFn(), funcID );

  // Prepare test quaternion.
  core_Quaternion Q(axis, angleRad);

  // Convert to Rodrigues vector.
  core_XYZ lambda;
  Q.AsRodrigues(lambda);

  // Convert back for verification.
  core_Quaternion QRef;
  QRef.SetRotationFromRodrigues(lambda);
  //
  if ( !Q.IsEqual(QRef, 1.e-6) )
    return res.failure();

  // Verify lambda.
  if ( (lambda - lambdaRef).Modulus() > 1.0e-6 )
    return res.failure();

  // Convert quaternion to a rotation matrix, then convert the obtained
  // Rodrigues vector to a rotation matrix, then compare the obtained matrices:
  // they should be equal.
  double mxQn[3][3], mxRodrigues[3][3];
  Q.AsMatrix3x3(mxQn);
  Q.FromRodriguesToMatrix3x3(lambda, mxRodrigues);

  // Verify.
  for ( int r = 0; r < 3; ++r )
    for ( int c = 0; c < 3; ++c )
      if ( fabs(mxQn[r][c] - mxRodrigues[r][c]) > 1.0e-6 )
        return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 001.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Quaternion::create(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  core_XYZ axis = core_XYZ::OZ();
  const double ang = 30*M_PI/180;
  core_Quaternion Q(axis, ang);

  const double q0_ref = 0.965925;
  const double qx_ref = 0.000000;
  const double qy_ref = 0.000000;
  const double qz_ref = 0.258819;

  SetVarDescr("qnAxis", axis, ID(), funcID);
  SetVarDescr("qnAngle", ang*180/M_PI, ID(), funcID);
  SetVarDescr("q0", Q.Q0(), ID(), funcID);
  SetVarDescr("qx", Q.Qx(), ID(), funcID);
  SetVarDescr("qy", Q.Qy(), ID(), funcID);
  SetVarDescr("qz", Q.Qz(), ID(), funcID);
  SetVarDescr("q0_ref", q0_ref, ID(), funcID);
  SetVarDescr("qx_ref", qx_ref, ID(), funcID);
  SetVarDescr("qy_ref", qy_ref, ID(), funcID);
  SetVarDescr("qz_ref", qz_ref, ID(), funcID);

  // Comparison tolerance
  const double tol = 1.0e-6;

  // Verification
  if ( fabs(q0_ref - Q.Q0()) > tol )
    return res.failure();
  if ( fabs(qx_ref - Q.Qx()) > tol )
    return res.failure();
  if ( fabs(qy_ref - Q.Qy()) > tol )
    return res.failure();
  if ( fabs(qz_ref - Q.Qz()) > tol )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 002.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Quaternion::add(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  core_XYZ axis1 = core_XYZ::OZ(), axis2 = core_XYZ::OX();
  const double ang1 = 60*M_PI/180, ang2 = 10*M_PI/180;
  core_Quaternion Q1(axis1, ang1), Q2(axis2, ang2);

  // Calculate
  core_Quaternion Q = Q1 + Q2;

  const double q0_ref = 1.862220;
  const double qx_ref = 0.087155;
  const double qy_ref = 0.000000;
  const double qz_ref = 0.500000;

  SetVarDescr("qn1Axis", axis1, ID(), funcID);
  SetVarDescr("qn1Angle", ang1*180/M_PI, ID(), funcID);
  SetVarDescr("qn2Axis", axis2, ID(), funcID);
  SetVarDescr("qn2Angle", ang2*180/M_PI, ID(), funcID);
  SetVarDescr("q0", Q.Q0(), ID(), funcID);
  SetVarDescr("qx", Q.Qx(), ID(), funcID);
  SetVarDescr("qy", Q.Qy(), ID(), funcID);
  SetVarDescr("qz", Q.Qz(), ID(), funcID);
  SetVarDescr("q0_ref", q0_ref, ID(), funcID);
  SetVarDescr("qx_ref", qx_ref, ID(), funcID);
  SetVarDescr("qy_ref", qy_ref, ID(), funcID);
  SetVarDescr("qz_ref", qz_ref, ID(), funcID);

  // Comparison tolerance
  const double tol = 1.0e-6;

  // Verification
  if ( fabs(q0_ref - Q.Q0()) > tol )
    return res.failure();
  if ( fabs(qx_ref - Q.Qx()) > tol )
    return res.failure();
  if ( fabs(qy_ref - Q.Qy()) > tol )
    return res.failure();
  if ( fabs(qz_ref - Q.Qz()) > tol )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 003.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Quaternion::subtract(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  core_XYZ axis1 = core_XYZ::OZ(), axis2 = core_XYZ::OX();
  const double ang1 = 60*M_PI/180, ang2 = 10*M_PI/180;
  core_Quaternion Q1(axis1, ang1), Q2(axis2, ang2);

  // Calculate
  core_Quaternion Q = Q1 - Q2;

  const double q0_ref = -0.130169;
  const double qx_ref = -0.087155;
  const double qy_ref =  0.000000;
  const double qz_ref =  0.500000;

  SetVarDescr("qn1Axis", axis1, ID(), funcID);
  SetVarDescr("qn1Angle", ang1*180/M_PI, ID(), funcID);
  SetVarDescr("qn2Axis", axis2, ID(), funcID);
  SetVarDescr("qn2Angle", ang2*180/M_PI, ID(), funcID);
  SetVarDescr("q0", Q.Q0(), ID(), funcID);
  SetVarDescr("qx", Q.Qx(), ID(), funcID);
  SetVarDescr("qy", Q.Qy(), ID(), funcID);
  SetVarDescr("qz", Q.Qz(), ID(), funcID);
  SetVarDescr("q0_ref", q0_ref, ID(), funcID);
  SetVarDescr("qx_ref", qx_ref, ID(), funcID);
  SetVarDescr("qy_ref", qy_ref, ID(), funcID);
  SetVarDescr("qz_ref", qz_ref, ID(), funcID);

  // Comparison tolerance
  const double tol = 1.0e-6;

  // Verification
  if ( fabs(q0_ref - Q.Q0()) > tol )
    return res.failure();
  if ( fabs(qx_ref - Q.Qx()) > tol )
    return res.failure();
  if ( fabs(qy_ref - Q.Qy()) > tol )
    return res.failure();
  if ( fabs(qz_ref - Q.Qz()) > tol )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 004.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Quaternion::product_qn(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  core_XYZ axis1 = core_XYZ::OZ(), axis2 = core_XYZ::OX();
  const double ang1 = 60*M_PI/180, ang2 = 10*M_PI/180;
  core_Quaternion Q1(axis1, ang1), Q2(axis2, ang2);

  // Calculate
  core_Quaternion Q = Q1 * Q2;

  const double q0_ref = 0.862729;
  const double qx_ref = 0.075479;
  const double qy_ref = 0.043577;
  const double qz_ref = 0.498097;

  SetVarDescr("qn1Axis", axis1, ID(), funcID);
  SetVarDescr("qn1Angle", ang1*180/M_PI, ID(), funcID);
  SetVarDescr("qn2Axis", axis2, ID(), funcID);
  SetVarDescr("qn2Angle", ang2*180/M_PI, ID(), funcID);
  SetVarDescr("q0", Q.Q0(), ID(), funcID);
  SetVarDescr("qx", Q.Qx(), ID(), funcID);
  SetVarDescr("qy", Q.Qy(), ID(), funcID);
  SetVarDescr("qz", Q.Qz(), ID(), funcID);
  SetVarDescr("q0_ref", q0_ref, ID(), funcID);
  SetVarDescr("qx_ref", qx_ref, ID(), funcID);
  SetVarDescr("qy_ref", qy_ref, ID(), funcID);
  SetVarDescr("qz_ref", qz_ref, ID(), funcID);

  // Comparison tolerance
  const double tol = 1.0e-6;

  // Verification
  if ( fabs(q0_ref - Q.Q0()) > tol )
    return res.failure();
  if ( fabs(qx_ref - Q.Qx()) > tol )
    return res.failure();
  if ( fabs(qy_ref - Q.Qy()) > tol )
    return res.failure();
  if ( fabs(qz_ref - Q.Qz()) > tol )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 005.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Quaternion::product_scalar(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  core_XYZ axis(1.0, 1.2, 1.4);
  const double ang = 135*M_PI/180, C = 10.0;
  core_Quaternion Q(axis, ang);

  // Calculate
  Q = Q*C;

  const double q0_ref = 3.826834;
  const double qx_ref = 4.404422;
  const double qy_ref = 5.285307;
  const double qz_ref = 6.166191;

  SetVarDescr("qnAxis", axis, ID(), funcID);
  SetVarDescr("qnAngle", ang*180/M_PI, ID(), funcID);
  SetVarDescr("C", C, ID(), funcID);
  SetVarDescr("q0", Q.Q0(), ID(), funcID);
  SetVarDescr("qx", Q.Qx(), ID(), funcID);
  SetVarDescr("qy", Q.Qy(), ID(), funcID);
  SetVarDescr("qz", Q.Qz(), ID(), funcID);
  SetVarDescr("q0_ref", q0_ref, ID(), funcID);
  SetVarDescr("qx_ref", qx_ref, ID(), funcID);
  SetVarDescr("qy_ref", qy_ref, ID(), funcID);
  SetVarDescr("qz_ref", qz_ref, ID(), funcID);

  // Comparison tolerance
  const double tol = 1.0e-6;

  // Verification
  if ( fabs(q0_ref - Q.Q0()) > tol )
    return res.failure();
  if ( fabs(qx_ref - Q.Qx()) > tol )
    return res.failure();
  if ( fabs(qy_ref - Q.Qy()) > tol )
    return res.failure();
  if ( fabs(qz_ref - Q.Qz()) > tol )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 006.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Quaternion::dot_product(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  core_XYZ axis1 = core_XYZ::OZ(), axis2 = core_XYZ::OX();
  const double ang1 = 60*M_PI/180, ang2 = 10*M_PI/180;
  core_Quaternion Q1(axis1, ang1), Q2(axis2, ang2);

  // Calculate
  const double dot = Q1.Dot(Q2);
  const double dot_ref = 0.862729;

  SetVarDescr("qn1Axis", axis1, ID(), funcID);
  SetVarDescr("qn1Angle", ang1*180/M_PI, ID(), funcID);
  SetVarDescr("qn2Axis", axis2, ID(), funcID);
  SetVarDescr("qn2Angle", ang2*180/M_PI, ID(), funcID);
  SetVarDescr("dot", dot, ID(), funcID);
  SetVarDescr("dot", dot_ref, ID(), funcID);

  // Comparison tolerance
  const double tol = 1.0e-6;

  // Verification
  if ( fabs(dot - dot_ref) > tol )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 007.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Quaternion::cross_product(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  core_XYZ axis1 = core_XYZ::OZ(), axis2 = core_XYZ::OX();
  const double ang1 = 60*M_PI/180, ang2 = 10*M_PI/180;
  core_Quaternion Q1(axis1, ang1), Q2(axis2, ang2);

  // Calculate
  core_Quaternion Q = Q1.Cross(Q2);

  const double q0_ref = 0.000000;
  const double qx_ref = 0.000000;
  const double qy_ref = 0.043577;
  const double qz_ref = 0.000000;

  SetVarDescr("qn1Axis", axis1, ID(), funcID);
  SetVarDescr("qn1Angle", ang1*180/M_PI, ID(), funcID);
  SetVarDescr("qn2Axis", axis2, ID(), funcID);
  SetVarDescr("qn2Angle", ang2*180/M_PI, ID(), funcID);
  SetVarDescr("q0", Q.Q0(), ID(), funcID);
  SetVarDescr("qx", Q.Qx(), ID(), funcID);
  SetVarDescr("qy", Q.Qy(), ID(), funcID);
  SetVarDescr("qz", Q.Qz(), ID(), funcID);
  SetVarDescr("q0_ref", q0_ref, ID(), funcID);
  SetVarDescr("qx_ref", qx_ref, ID(), funcID);
  SetVarDescr("qy_ref", qy_ref, ID(), funcID);
  SetVarDescr("qz_ref", qz_ref, ID(), funcID);

  // Comparison tolerance
  const double tol = 1.0e-6;

  // Verification
  if ( fabs(q0_ref - Q.Q0()) > tol )
    return res.failure();
  if ( fabs(qx_ref - Q.Qx()) > tol )
    return res.failure();
  if ( fabs(qy_ref - Q.Qy()) > tol )
    return res.failure();
  if ( fabs(qz_ref - Q.Qz()) > tol )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 008.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Quaternion::invert(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  core_XYZ axis = core_XYZ::OZ();
  const double ang = 30*M_PI/180;
  core_Quaternion Q(axis, ang);

  // Calculate
  core_Quaternion P = Q.Inverted();
  core_Quaternion R = Q * P;

  const double p0_ref =  0.965925;
  const double px_ref =  0.000000;
  const double py_ref =  0.000000;
  const double pz_ref = -0.258819;

  const double r0_ref = 1.000000;
  const double rx_ref = 0.000000;
  const double ry_ref = 0.000000;
  const double rz_ref = 0.000000;

  SetVarDescr("qnAxis", axis, ID(), funcID);
  SetVarDescr("qnAngle", ang*180/M_PI, ID(), funcID);
  SetVarDescr("p0", P.Q0(), ID(), funcID);
  SetVarDescr("px", P.Qx(), ID(), funcID);
  SetVarDescr("py", P.Qy(), ID(), funcID);
  SetVarDescr("pz", P.Qz(), ID(), funcID);
  SetVarDescr("p0_ref", p0_ref, ID(), funcID);
  SetVarDescr("px_ref", px_ref, ID(), funcID);
  SetVarDescr("py_ref", py_ref, ID(), funcID);
  SetVarDescr("pz_ref", pz_ref, ID(), funcID);
  SetVarDescr("r0", R.Q0(), ID(), funcID);
  SetVarDescr("rx", R.Qx(), ID(), funcID);
  SetVarDescr("ry", R.Qy(), ID(), funcID);
  SetVarDescr("rz", R.Qz(), ID(), funcID);
  SetVarDescr("r0_ref", r0_ref, ID(), funcID);
  SetVarDescr("rx_ref", rx_ref, ID(), funcID);
  SetVarDescr("ry_ref", ry_ref, ID(), funcID);
  SetVarDescr("rz_ref", rz_ref, ID(), funcID);

  // Comparison tolerance
  const double tol = 1.0e-6;

  // Verification
  if ( fabs(p0_ref - P.Q0()) > tol )
    return res.failure();
  if ( fabs(px_ref - P.Qx()) > tol )
    return res.failure();
  if ( fabs(py_ref - P.Qy()) > tol )
    return res.failure();
  if ( fabs(pz_ref - P.Qz()) > tol )
    return res.failure();
  if ( fabs(r0_ref - R.Q0()) > tol )
    return res.failure();
  if ( fabs(rx_ref - R.Qx()) > tol )
    return res.failure();
  if ( fabs(ry_ref - R.Qy()) > tol )
    return res.failure();
  if ( fabs(rz_ref - R.Qz()) > tol )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 009.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Quaternion::conjugate(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  core_XYZ axis = core_XYZ::OZ();
  const double ang = 30*M_PI/180;
  core_Quaternion Q(axis, ang);

  // Calculate
  Q.Conjugate();

  const double q0_ref =  0.965925;
  const double qx_ref =  0.000000;
  const double qy_ref =  0.000000;
  const double qz_ref = -0.258819;

  SetVarDescr("qnAxis", axis, ID(), funcID);
  SetVarDescr("qnAngle", ang*180/M_PI, ID(), funcID);
  SetVarDescr("q0", Q.Q0(), ID(), funcID);
  SetVarDescr("qx", Q.Qx(), ID(), funcID);
  SetVarDescr("qy", Q.Qy(), ID(), funcID);
  SetVarDescr("qz", Q.Qz(), ID(), funcID);
  SetVarDescr("q0_ref", q0_ref, ID(), funcID);
  SetVarDescr("qx_ref", qx_ref, ID(), funcID);
  SetVarDescr("qy_ref", qy_ref, ID(), funcID);
  SetVarDescr("qz_ref", qz_ref, ID(), funcID);

  // Comparison tolerance
  const double tol = 1.0e-6;

  // Verification
  if ( fabs(q0_ref - Q.Q0()) > tol )
    return res.failure();
  if ( fabs(qx_ref - Q.Qx()) > tol )
    return res.failure();
  if ( fabs(qy_ref - Q.Qy()) > tol )
    return res.failure();
  if ( fabs(qz_ref - Q.Qz()) > tol )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 010.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Quaternion::to_matrix(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  core_XYZ axis = core_XYZ::OZ();
  const double ang = 30*M_PI/180;
  core_Quaternion Q(axis, ang);

  // Calculate
  double mx[3][3], m_ref[3][3];
  Q.AsMatrix3x3(mx);

  m_ref[0][0] =  0.866025;
  m_ref[0][1] = -0.500000;
  m_ref[0][2] =  0.000000;
  m_ref[1][0] =  0.500000;
  m_ref[1][1] =  0.866025;
  m_ref[1][2] =  0.000000;
  m_ref[2][0] =  0.000000;
  m_ref[2][1] =  0.000000;
  m_ref[2][2] =  1.000000;

  SetVarDescr("qnAxis", axis, ID(), funcID);
  SetVarDescr("qnAngle", ang*180/M_PI, ID(), funcID);

  // Comparison tolerance
  const double tol = 1.0e-6;

  // Verification
  for ( int r = 0; r < 3; ++r )
    for ( int c = 0; c < 3; ++c )
      if ( fabs(m_ref[r][c] - mx[r][c]) > tol )
        return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Tests conversion to Rodrigues form.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Quaternion::to_rodrigues01(const int funcID)
{
  return testRodrigues( core_XYZ::OZ(),
                        30*M_PI/180,
                        core_XYZ(0., 0., 0.26794919243112270),
                        funcID );
}

//-----------------------------------------------------------------------------

//! Tests conversion to Rodrigues form.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Quaternion::to_rodrigues02(const int funcID)
{
  return testRodrigues( core_XYZ(1., 0.5, 0.25),
                        60*M_PI/180,
                        core_XYZ(0.50395263067896956, 0.25197631533948478, 0.12598815766974239),
                        funcID );
}
