//-----------------------------------------------------------------------------
// Created on: 26 July 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2018-present, Sergey Slyadnev
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
#include <mobius/test_Integral.h>

// BSpl includes
#include <mobius/bspl_N.h>

// Core includes
#include <mobius/core_Integral.h>
#include <mobius/core_UnivariateFunc.h>

// Standard includes
#include <math.h>

//-----------------------------------------------------------------------------

namespace mobius {

/*****************************************************************************/

//! Test integrand as a B-spline function N_{i,p} defined on a knot vector U.
class test_Integral_N : public core_UnivariateFunc
{
public:

  //! Complete ctor.
  test_Integral_N(const int                  i,
                  const int                  p,
                  const std::vector<adouble>& U)
  //
  : core_UnivariateFunc (),
    m_iIndex            (i),
    m_iDegree           (p),
    m_U                 (U)
  {}

public:

  virtual adouble Eval(const adouble x) const
  {
    const adouble
      val = bspl_N()(x, m_U, m_iDegree, m_iIndex);
    //
    return val;
  }

private:

  test_Integral_N(const test_Integral_N&) = delete;
  void operator=(const test_Integral_N&) = delete;

private:

  int                        m_iIndex;  //!< 0-based B-spline function index.
  int                        m_iDegree; //!< B-spline degree.
  const std::vector<adouble>& m_U;       //!< Knots.

};

/*****************************************************************************/

//! Test integrand as a product of two B-spline functions N_{i,p}*N_{j,p}
//! defined on a knot vector U.
class test_Integral_NN : public core_UnivariateFunc
{
public:

  //! Complete ctor.
  test_Integral_NN(const int                  i,
                   const int                  j,
                   const int                  p,
                   const std::vector<adouble>& U)
  //
  : core_UnivariateFunc (),
    m_iIndex1           (i),
    m_iIndex2           (j),
    m_iDegree           (p),
    m_U                 (U)
  {}

public:

  virtual adouble Eval(const adouble x) const
  {
    const adouble
      val1 = bspl_N()(x, m_U, m_iDegree, m_iIndex1);
    //
    const adouble
      val2 = bspl_N()(x, m_U, m_iDegree, m_iIndex2);
    //
    return val1*val2;
  }

private:

  test_Integral_NN(const test_Integral_NN&) = delete;
  void operator=(const test_Integral_NN&) = delete;

private:

  int                        m_iIndex1; //!< 0-based B-spline function index.
  int                        m_iIndex2; //!< 0-based B-spline function index.
  int                        m_iDegree; //!< B-spline degree.
  const std::vector<adouble>& m_U;       //!< Knots.

};

/*****************************************************************************/

//! Test integrand as multivariate const.
class test_Integral_MultiConst : public core_TwovariateFunc
{
public:

  //! Complete ctor.
  test_Integral_MultiConst(const adouble val)
  //
  : core_TwovariateFunc (),
    m_fVal              (val)
  {}

public:

  virtual adouble Eval(const adouble, const adouble) const
  {
    return m_fVal;
  }

private:

  test_Integral_MultiConst(const test_Integral_MultiConst&) = delete;
  void operator=(const test_Integral_MultiConst&) = delete;

private:

  adouble m_fVal; //!< Const value of the function.

};

/*****************************************************************************/

//! Test integrand as multivariate function {x y^2}.
class test_Integral_XYSquared : public core_TwovariateFunc
{
public:

  //! Complete ctor.
  test_Integral_XYSquared() : core_TwovariateFunc()
  {}

public:

  virtual adouble Eval(const adouble x, const adouble y) const
  {
    return x*y*y;
  }

private:

  test_Integral_XYSquared(const test_Integral_XYSquared&) = delete;
  void operator=(const test_Integral_XYSquared&) = delete;

};

/*****************************************************************************/

//! Test integrand as multivariate function {1 + 8xy}.
class test_Integral_Custom1 : public core_TwovariateFunc
{
public:

  //! Complete ctor.
  test_Integral_Custom1() : core_TwovariateFunc()
  {}

public:

  virtual adouble Eval(const adouble x, const adouble y) const
  {
    return 1 + 8*x*y;
  }

private:

  test_Integral_Custom1(const test_Integral_Custom1&) = delete;
  void operator=(const test_Integral_Custom1&) = delete;

};

/*****************************************************************************/

//! Test integrand as multivariate function {y sin(x)}.
class test_Integral_Custom2 : public core_TwovariateFunc
{
public:

  //! Complete ctor.
  test_Integral_Custom2() : core_TwovariateFunc()
  {}

public:

  virtual adouble Eval(const adouble x, const adouble y) const
  {
    return y * sin(x);
  }

private:

  test_Integral_Custom2(const test_Integral_Custom2&) = delete;
  void operator=(const test_Integral_Custom2&) = delete;

};

/*****************************************************************************/

//! Test integrand as multivariate function {4 - x - y}.
class test_Integral_Custom3 : public core_TwovariateFunc
{
public:

  //! Complete ctor.
  test_Integral_Custom3() : core_TwovariateFunc()
  {}

public:

  virtual adouble Eval(const adouble x, const adouble y) const
  {
    return 4 - x - y;
  }

private:

  test_Integral_Custom3(const test_Integral_Custom3&) = delete;
  void operator=(const test_Integral_Custom2&) = delete;

};

/*****************************************************************************/

};

//-----------------------------------------------------------------------------

//! Test scenario 001.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome mobius::test_Integral::test01(const int funcID)
{
  outcome res( DescriptionFn() );

  const std::vector<adouble> U         = {0, 0, 0.2, 0.5, 1, 1};
  const int                 p         = 1;
  const int                 nRectBins = 1000;
  const int                 nGaussPts = 6;
  const adouble              refRect   = 0.1;
  const adouble              refGauss1 = 0.0988027;
  const adouble              refGauss2 = 0.1;
  const adouble              prec      = 1.0e-6;
  const adouble              toler     = 1.0e-3;

  test_Integral_N F(0, p, U);

  // Evaluate using rectangle integration.
  const adouble
    rectVal = core_Integral::ComputeRect(&F, U[0], U[U.size() - 1], nRectBins);

  std::cout << "***" << std::endl;
  std::cout << "rectVal = " << rectVal << std::endl;

  // Evaluate using Gaussian integration.
  const adouble
    gauss1Val = core_Integral::gauss::Compute(&F, U[0], U[U.size() - 1], nGaussPts);
  std::cout << "coarse gaussVal [" << nGaussPts << "] = " << gauss1Val << std::endl;

  // Evaluate using Gaussian integration in knot intervals.
  adouble precGaussVal = 0;
  for ( size_t k = 0; k < U.size() - 1; ++k )
  {
    const adouble
      intervGaussVal = core_Integral::gauss::Compute(&F, U[k], U[k+1], nGaussPts);
    //
    precGaussVal += intervGaussVal;
  }
  std::cout << "interval gaussVal [" << nGaussPts << "] = " << precGaussVal << std::endl;

  // Verify.
  if ( abs(refGauss1.getValue() - gauss1Val.getValue()) > prec )
    return res.failure();
  //
  if ( abs(refGauss2.getValue() - precGaussVal.getValue()) > prec )
    return res.failure();
  //
  if ( abs(precGaussVal.getValue() - refRect.getValue()) > toler )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 002.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome mobius::test_Integral::test02(const int funcID)
{
  outcome res( DescriptionFn() );

  const std::vector<adouble> U         = {0, 0, 0, 0.2, 0.5, 1, 1, 1};
  const int                 p         = 2;
  const int                 nRectBins = 1000;
  const int                 nGaussPts = 6;
  const adouble              refRect   = 0.0666663;
  const adouble              refGauss1 = 0.0634036;
  const adouble              refGauss2 = 0.0666667;
  const adouble              prec      = 1.0e-6;
  const adouble              toler     = 1.0e-3;

  test_Integral_N F(0, p, U);

  // Evaluate using rectangle integration.
  const adouble
    rectVal = core_Integral::ComputeRect(&F, U[0], U[U.size() - 1], nRectBins);

  std::cout << "***" << std::endl;
  std::cout << "rectVal = " << rectVal << std::endl;

  // Evaluate using Gaussian integration.
  const adouble
    gauss1Val = core_Integral::gauss::Compute(&F, U[0], U[U.size() - 1], nGaussPts);
  std::cout << "coarse gaussVal [" << nGaussPts << "] = " << gauss1Val << std::endl;

  // Evaluate using Gaussian integration in knot intervals.
  adouble precGaussVal = 0;
  for ( size_t k = 0; k < U.size() - 1; ++k )
  {
    const adouble
      intervGaussVal = core_Integral::gauss::Compute(&F, U[k], U[k+1], nGaussPts);
    //
    precGaussVal += intervGaussVal;
  }
  std::cout << "interval gaussVal [" << nGaussPts << "] = " << precGaussVal << std::endl;

  // Verify.
  if ( abs(refGauss1.getValue() - gauss1Val.getValue()) > prec )
    return res.failure();
  //
  if ( abs(refGauss2.getValue() - precGaussVal.getValue()) > prec )
    return res.failure();
  //
  if ( abs(precGaussVal.getValue() - refRect.getValue()) > toler )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 003.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome mobius::test_Integral::test03(const int funcID)
{
  outcome res( DescriptionFn() );

  const std::vector<adouble> U         = {0, 0, 0, 0, 0.2, 0.5, 1, 1, 1, 1};
  const int                 p         = 3;
  const int                 nRectBins = 1000;
  const int                 nGaussPts = 6;
  const adouble              refRect   = 0.125001;
  const adouble              refGauss1 = 0.124902;
  const adouble              refGauss2 = 0.125;
  const adouble              prec      = 1.0e-6;
  const adouble              toler     = 1.0e-3;

  test_Integral_N F(1, p, U);

  // Evaluate using rectangle integration.
  const adouble
    rectVal = core_Integral::ComputeRect(&F, U[0], U[U.size() - 1], nRectBins);

  std::cout << "***" << std::endl;
  std::cout << "rectVal = " << rectVal << std::endl;

  // Evaluate using Gaussian integration.
  const adouble
    gauss1Val = core_Integral::gauss::Compute(&F, U[0], U[U.size() - 1], nGaussPts);
  std::cout << "coarse gaussVal [" << nGaussPts << "] = " << gauss1Val << std::endl;

  // Evaluate using Gaussian integration in knot intervals.
  adouble precGaussVal = 0;
  for ( size_t k = 0; k < U.size() - 1; ++k )
  {
    const adouble
      intervGaussVal = core_Integral::gauss::Compute(&F, U[k], U[k+1], nGaussPts);
    //
    precGaussVal += intervGaussVal;
  }
  std::cout << "interval gaussVal [" << nGaussPts << "] = " << precGaussVal << std::endl;

  // Verify.
  if ( abs(refGauss1.getValue() - gauss1Val.getValue()) > prec )
    return res.failure();
  //
  if ( abs(refGauss2.getValue() - precGaussVal.getValue()) > prec )
    return res.failure();
  //
  if ( abs(precGaussVal.getValue() - refRect.getValue()) > toler )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 004.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome mobius::test_Integral::test04(const int funcID)
{
  outcome res( DescriptionFn() );

  const std::vector<adouble> U         = {0, 0, 0, 0, 0, 0.2, 0.5, 1, 1, 1, 1, 1};
  const int                 p         = 4;
  const int                 nRectBins = 1000;
  const int                 nGaussPts = 6;
  const adouble              refRect   = 0.2;
  const adouble              refGauss1 = 0.199795;
  const adouble              refGauss2 = 0.2;
  const adouble              prec      = 1.0e-6;
  const adouble              toler     = 1.0e-3;

  test_Integral_N F(3, p, U);

  // Evaluate using rectangle integration.
  const adouble
    rectVal = core_Integral::ComputeRect(&F, U[0], U[U.size() - 1], nRectBins);

  std::cout << "***" << std::endl;
  std::cout << "rectVal = " << rectVal << std::endl;

  // Evaluate using Gaussian integration.
  const adouble
    gauss1Val = core_Integral::gauss::Compute(&F, U[0], U[U.size() - 1], nGaussPts);
  std::cout << "coarse gaussVal [" << nGaussPts << "] = " << gauss1Val << std::endl;

  // Evaluate using Gaussian integration in knot intervals.
  adouble precGaussVal = 0;
  for ( size_t k = 0; k < U.size() - 1; ++k )
  {
    const adouble
      intervGaussVal = core_Integral::gauss::Compute(&F, U[k], U[k+1], nGaussPts);
    //
    precGaussVal += intervGaussVal;
  }
  std::cout << "interval gaussVal [" << nGaussPts << "] = " << precGaussVal << std::endl;

  // Verify.
  if ( abs(refGauss1.getValue() - gauss1Val.getValue()) > prec )
    return res.failure();
  //
  if ( abs(refGauss2.getValue() - precGaussVal.getValue()) > prec )
    return res.failure();
  //
  if ( abs(precGaussVal.getValue() - refRect.getValue()) > toler )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 005.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome mobius::test_Integral::test05(const int funcID)
{
  outcome res( DescriptionFn() );

  const std::vector<adouble> U         = {0, 0, 0, 0, 0, 0, 0.3, 0.4, 0.5, 0.8, 1, 1, 1, 1, 1, 1};
  const int                 p         = 5;
  const int                 nRectBins = 1000;
  const int                 nGaussPts = 6;
  const adouble              refRect   = 0.166667;
  const adouble              refGauss1 = 0.164276;
  const adouble              refGauss2 = 0.166667;
  const adouble              prec      = 1.0e-6;
  const adouble              toler     = 1.0e-3;

  test_Integral_N F(5, p, U);

  // Evaluate using rectangle integration.
  const adouble
    rectVal = core_Integral::ComputeRect(&F, U[0], U[U.size() - 1], nRectBins);

  std::cout << "***" << std::endl;
  std::cout << "rectVal = " << rectVal << std::endl;

  // Evaluate using Gaussian integration.
  const adouble
    gauss1Val = core_Integral::gauss::Compute(&F, U[0], U[U.size() - 1], nGaussPts);
  std::cout << "coarse gaussVal [" << nGaussPts << "] = " << gauss1Val << std::endl;

  // Evaluate using Gaussian integration in knot intervals.
  adouble precGaussVal = 0;
  for ( size_t k = 0; k < U.size() - 1; ++k )
  {
    const adouble
      intervGaussVal = core_Integral::gauss::Compute(&F, U[k], U[k+1], nGaussPts);
    //
    precGaussVal += intervGaussVal;
  }
  std::cout << "interval gaussVal [" << nGaussPts << "] = " << precGaussVal << std::endl;

  // Verify.
  if ( abs(refGauss1.getValue() - gauss1Val.getValue()) > prec )
    return res.failure();
  //
  if ( abs(refGauss2.getValue() - precGaussVal.getValue()) > prec )
    return res.failure();
  //
  if ( abs(precGaussVal.getValue() - refRect.getValue()) > toler )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 006.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome mobius::test_Integral::test06(const int funcID)
{
  outcome res( DescriptionFn() );

  const std::vector<adouble> U         = {0, 0, 0, 0, 0, 0, 0.3, 0.4, 0.5, 0.8, 1, 1, 1, 1, 1, 1};
  const int                 p         = 5;
  const int                 nRectBins = 1000;
  const int                 nGaussPts = 6;
  const adouble              refRect   = 0.0242081;
  const adouble              refGauss1 = 0.0243687;
  const adouble              refGauss2 = 0.0242081;
  const adouble              prec      = 1.0e-6;
  const adouble              toler     = 1.0e-3;

  test_Integral_NN F(3, 2, p, U);

  // Evaluate using rectangle integration.
  const adouble
    rectVal = core_Integral::ComputeRect(&F, U[0], U[U.size() - 1], nRectBins);

  std::cout << "***" << std::endl;
  std::cout << "rectVal = " << rectVal << std::endl;

  // Evaluate using Gaussian integration.
  const adouble
    gauss1Val = core_Integral::gauss::Compute(&F, U[0], U[U.size() - 1], nGaussPts);
  std::cout << "coarse gaussVal [" << nGaussPts << "] = " << gauss1Val << std::endl;

  // Evaluate using Gaussian integration in knot intervals.
  adouble precGaussVal = 0;
  for ( size_t k = 0; k < U.size() - 1; ++k )
  {
    const adouble
      intervGaussVal = core_Integral::gauss::Compute(&F, U[k], U[k+1], nGaussPts);
    //
    precGaussVal += intervGaussVal;
  }
  std::cout << "interval gaussVal [" << nGaussPts << "] = " << precGaussVal << std::endl;

  // Verify.
  if ( abs(refGauss1.getValue() - gauss1Val.getValue()) > prec )
    return res.failure();
  //
  if ( abs(refGauss2.getValue() - precGaussVal.getValue()) > prec )
    return res.failure();
  //
  if ( abs(precGaussVal.getValue() - refRect.getValue()) > toler )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 007. Numerically evaluates adouble integral with known
//! reference value which can be calculated analytically.
//!
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome mobius::test_Integral::test07(const int funcID)
{
  outcome res( DescriptionFn() );

  const adouble           fVal   = 15.0;
  const std::vector<int> order  = {3, 3};
  const adouble           refVal = fVal*4;
  const adouble           prec   = 1e-6;

  test_Integral_MultiConst F(fVal);

  // Evaluate.
  const adouble
    gaussVal = core_Integral::gauss::Compute(&F, 0, 2, 0, 2, order[0], order[1]);

  std::cout << "***" << std::endl;
  std::cout << "gaussVal = " << gaussVal << std::endl;

  if ( abs(gaussVal.getValue() - refVal.getValue()) > prec )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 008. Numerically evaluates adouble integral with known
//! reference value which can be calculated analytically.
//!
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome mobius::test_Integral::test08(const int funcID)
{
  outcome res( DescriptionFn() );

  const std::vector<int> order  = {8, 8};
  const adouble           refVal = 2./3.;
  const adouble           prec   = 1e-6;

  test_Integral_XYSquared F;

  // Evaluate.
  const adouble
    gaussVal = core_Integral::gauss::Compute(&F, 0, 2, 0, 1, order[0], order[1]);

  std::cout << "***" << std::endl;
  std::cout << "gaussVal = " << gaussVal << std::endl;

  if ( abs(gaussVal.getValue() - refVal.getValue()) > prec )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 009. Numerically evaluates adouble integral with known
//! reference value which can be calculated analytically.
//!
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome mobius::test_Integral::test09(const int funcID)
{
  outcome res( DescriptionFn() );

  const std::vector<int> order  = {6, 6};
  const adouble           refVal = 57.0;
  const adouble           prec   = 1e-6;

  test_Integral_Custom1 F;

  // Evaluate.
  const adouble
    gaussVal = core_Integral::gauss::Compute(&F, 0, 3, 1, 2, order[0], order[1]);

  std::cout << "***" << std::endl;
  std::cout << "gaussVal = " << gaussVal << std::endl;

  if ( abs(gaussVal.getValue() - refVal.getValue()) > prec )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 010. Numerically evaluates adouble integral with known
//! reference value which can be calculated analytically.
//!
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome mobius::test_Integral::test10(const int funcID)
{
  outcome res( DescriptionFn() );

  const std::vector<int> order  = {6, 6};
  const adouble           refVal = 1./2.;
  const adouble           prec   = 1e-6;

  test_Integral_Custom2 F;

  // Evaluate.
  const adouble
    gaussVal = core_Integral::gauss::Compute(&F, 0, M_PI/2, 0, 1, order[0], order[1]);

  std::cout << "***" << std::endl;
  std::cout << "gaussVal = " << gaussVal << std::endl;

  if ( abs(gaussVal.getValue() - refVal.getValue()) > prec )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 011. Numerically evaluates adouble integral with known
//! reference value which can be calculated analytically.
//!
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome mobius::test_Integral::test11(const int funcID)
{
  outcome res( DescriptionFn() );

  const std::vector<int> order  = {6, 6};
  const adouble           refVal = 5.;
  const adouble           prec   = 1e-6;

  test_Integral_Custom3 F;

  // Evaluate.
  const adouble
    gaussVal = core_Integral::gauss::Compute(&F, 0, 1, 0, 2, order[0], order[1]);

  std::cout << "***" << std::endl;
  std::cout << "gaussVal = " << gaussVal << std::endl;

  if ( abs(gaussVal.getValue() - refVal.getValue()) > prec )
    return res.failure();

  return res.success();
}
