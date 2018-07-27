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

//-----------------------------------------------------------------------------

namespace mobius {

//! Test integrand as a B-spline function N_{i,p} defined on a knot vector U.
class test_Integral_N : public core_UnivariateFunc
{
public:

  //! Complete ctor.
  test_Integral_N(const int                  i,
                  const int                  p,
                  const std::vector<double>& U)
  //
  : core_UnivariateFunc (),
    m_iIndex            (i),
    m_iDegree           (p),
    m_U                 (U)
  {}

public:

  virtual double Eval(const double x) const
  {
    const double
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
  const std::vector<double>& m_U;       //!< Knots.

};

//! Test integrand as a product of two B-spline functions N_{i,p}*N_{j,p}
//! defined on a knot vector U.
class test_Integral_NN : public core_UnivariateFunc
{
public:

  //! Complete ctor.
  test_Integral_NN(const int                  i,
                   const int                  j,
                   const int                  p,
                   const std::vector<double>& U)
  //
  : core_UnivariateFunc (),
    m_iIndex1           (i),
    m_iIndex2           (j),
    m_iDegree           (p),
    m_U                 (U)
  {}

public:

  virtual double Eval(const double x) const
  {
    const double
      val1 = bspl_N()(x, m_U, m_iDegree, m_iIndex1);
    //
    const double
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
  const std::vector<double>& m_U;       //!< Knots.

};

};

//-----------------------------------------------------------------------------

//! Test scenario 001.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome mobius::test_Integral::test01(const int funcID)
{
  outcome res( DescriptionFn() );

  const std::vector<double> U         = {0, 0, 0.2, 0.5, 1, 1};
  const int                 p         = 1;
  const int                 nRectBins = 1000;

  test_Integral_N F(0, p, U);

  // Evaluate using rectangle integration.
  const double
    rectVal = core_Integral::ComputeRect(&F, U[0], U[U.size() - 1], nRectBins);

  std::cout << "***" << std::endl;
  std::cout << "rectVal = " << rectVal << std::endl;

  // Evaluate using Gaussian integration.
  for ( int n = 1; n <= core_Integral::gauss::GetPointsMax(); ++n )
  {
    const int nGaussPts = n;

    const double
      gaussVal = core_Integral::gauss::Compute(&F, U[0], U[U.size() - 1], nGaussPts);

    std::cout << "gaussVal [" << nGaussPts << "] = " << gaussVal << std::endl;
  }

  // Evaluate using Gaussian integration in knot intervals.
  for ( int n = 1; n <= core_Integral::gauss::GetPointsMax(); ++n )
  {
    const int nGaussPts = n;

    double val = 0;
    for ( size_t k = 0; k < U.size() - 1; ++k )
    {
      const double
        gaussVal = core_Integral::gauss::Compute(&F, U[k], U[k+1], nGaussPts);
      //
      val += gaussVal;
    }

    std::cout << "interval gaussVal [" << nGaussPts << "] = " << val << std::endl;
  }

  // TODO: NYI

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 002.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome mobius::test_Integral::test02(const int funcID)
{
  outcome res( DescriptionFn() );

  const std::vector<double> U         = {0, 0, 0, 0.2, 0.5, 1, 1, 1};
  const int                 p         = 2;
  const int                 nRectBins = 1000;

  test_Integral_N F(0, p, U);

  // Evaluate using rectangle integration.
  const double
    rectVal = core_Integral::ComputeRect(&F, U[0], U[U.size() - 1], nRectBins);

  std::cout << "***" << std::endl;
  std::cout << "rectVal = " << rectVal << std::endl;

  // Evaluate using Gaussian integration.
  for ( int n = 1; n <= core_Integral::gauss::GetPointsMax(); ++n )
  {
    const int nGaussPts = n;

    const double
      gaussVal = core_Integral::gauss::Compute(&F, U[0], U[U.size() - 1], nGaussPts);

    std::cout << "gaussVal [" << nGaussPts << "] = " << gaussVal << std::endl;
  }

  // Evaluate using Gaussian integration in knot intervals.
  for ( int n = 1; n <= core_Integral::gauss::GetPointsMax(); ++n )
  {
    const int nGaussPts = n;

    double val = 0;
    for ( size_t k = 0; k < U.size() - 1; ++k )
    {
      const double
        gaussVal = core_Integral::gauss::Compute(&F, U[k], U[k+1], nGaussPts);
      //
      val += gaussVal;
    }

    std::cout << "interval gaussVal [" << nGaussPts << "] = " << val << std::endl;
  }

  // TODO: NYI

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 003.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome mobius::test_Integral::test03(const int funcID)
{
  outcome res( DescriptionFn() );

  const std::vector<double> U         = {0, 0, 0, 0, 0.2, 0.5, 1, 1, 1, 1};
  const int                 p         = 3;
  const int                 nRectBins = 1000;

  test_Integral_N F(1, p, U);

  // Evaluate using rectangle integration.
  const double
    rectVal = core_Integral::ComputeRect(&F, U[0], U[U.size() - 1], nRectBins);

  std::cout << "***" << std::endl;
  std::cout << "rectVal = " << rectVal << std::endl;

  // Evaluate using Gaussian integration.
  for ( int n = 1; n <= core_Integral::gauss::GetPointsMax(); ++n )
  {
    const int nGaussPts = n;

    const double
      gaussVal = core_Integral::gauss::Compute(&F, U[0], U[U.size() - 1], nGaussPts);

    std::cout << "gaussVal [" << nGaussPts << "] = " << gaussVal << std::endl;
  }

  // Evaluate using Gaussian integration in knot intervals.
  for ( int n = 1; n <= core_Integral::gauss::GetPointsMax(); ++n )
  {
    const int nGaussPts = n;

    double val = 0;
    for ( size_t k = 0; k < U.size() - 1; ++k )
    {
      const double
        gaussVal = core_Integral::gauss::Compute(&F, U[k], U[k+1], nGaussPts);
      //
      val += gaussVal;
    }

    std::cout << "interval gaussVal [" << nGaussPts << "] = " << val << std::endl;
  }

  // TODO: NYI

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 004.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome mobius::test_Integral::test04(const int funcID)
{
  outcome res( DescriptionFn() );

  const std::vector<double> U         = {0, 0, 0, 0, 0, 0.2, 0.5, 1, 1, 1, 1, 1};
  const int                 p         = 4;
  const int                 nRectBins = 1000;

  test_Integral_N F(3, p, U);

  // Evaluate using rectangle integration.
  const double
    rectVal = core_Integral::ComputeRect(&F, U[0], U[U.size() - 1], nRectBins);

  std::cout << "***" << std::endl;
  std::cout << "rectVal = " << rectVal << std::endl;

  // Evaluate using Gaussian integration.
  for ( int n = 1; n <= core_Integral::gauss::GetPointsMax(); ++n )
  {
    const int nGaussPts = n;

    const double
      gaussVal = core_Integral::gauss::Compute(&F, U[0], U[U.size() - 1], nGaussPts);

    std::cout << "gaussVal [" << nGaussPts << "] = " << gaussVal << std::endl;
  }

  // Evaluate using Gaussian integration in knot intervals.
  for ( int n = 1; n <= core_Integral::gauss::GetPointsMax(); ++n )
  {
    const int nGaussPts = n;

    double val = 0;
    for ( size_t k = 0; k < U.size() - 1; ++k )
    {
      const double
        gaussVal = core_Integral::gauss::Compute(&F, U[k], U[k+1], nGaussPts);
      //
      val += gaussVal;
    }

    std::cout << "interval gaussVal [" << nGaussPts << "] = " << val << std::endl;
  }

  // TODO: NYI

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 005.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome mobius::test_Integral::test05(const int funcID)
{
  outcome res( DescriptionFn() );

  const std::vector<double> U         = {0, 0, 0, 0, 0, 0, 0.3, 0.4, 0.5, 0.8, 1, 1, 1, 1, 1, 1};
  const int                 p         = 5;
  const int                 nRectBins = 1000;

  test_Integral_N F(5, p, U);

  // Evaluate using rectangle integration.
  const double
    rectVal = core_Integral::ComputeRect(&F, U[0], U[U.size() - 1], nRectBins);

  std::cout << "***" << std::endl;
  std::cout << "rectVal = " << rectVal << std::endl;

  // Evaluate using Gaussian integration.
  for ( int n = 1; n <= core_Integral::gauss::GetPointsMax(); ++n )
  {
    const int nGaussPts = n;

    const double
      gaussVal = core_Integral::gauss::Compute(&F, U[0], U[U.size() - 1], nGaussPts);

    std::cout << "gaussVal [" << nGaussPts << "] = " << gaussVal << std::endl;
  }

  // Evaluate using Gaussian integration in knot intervals.
  for ( int n = 1; n <= core_Integral::gauss::GetPointsMax(); ++n )
  {
    const int nGaussPts = n;

    double val = 0;
    for ( size_t k = 0; k < U.size() - 1; ++k )
    {
      const double
        gaussVal = core_Integral::gauss::Compute(&F, U[k], U[k+1], nGaussPts);
      //
      val += gaussVal;
    }

    std::cout << "interval gaussVal [" << nGaussPts << "] = " << val << std::endl;
  }

  // TODO: NYI

  return res.success();
}

//-----------------------------------------------------------------------------

//! Test scenario 006.
//! \param[in] funcID ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome mobius::test_Integral::test06(const int funcID)
{
  outcome res( DescriptionFn() );

  const std::vector<double> U         = {0, 0, 0, 0, 0, 0, 0.3, 0.4, 0.5, 0.8, 1, 1, 1, 1, 1, 1};
  const int                 p         = 5;
  const int                 nRectBins = 1000;

  test_Integral_NN F(3, 2, p, U);

  // Evaluate using rectangle integration.
  const double
    rectVal = core_Integral::ComputeRect(&F, U[0], U[U.size() - 1], nRectBins);

  std::cout << "***" << std::endl;
  std::cout << "rectVal = " << rectVal << std::endl;

  // Evaluate using Gaussian integration.
  for ( int n = 1; n <= core_Integral::gauss::GetPointsMax(); ++n )
  {
    const int nGaussPts = n;

    const double
      gaussVal = core_Integral::gauss::Compute(&F, U[0], U[U.size() - 1], nGaussPts);

    std::cout << "gaussVal [" << nGaussPts << "] = " << gaussVal << std::endl;
  }

  // Evaluate using Gaussian integration in knot intervals.
  for ( int n = 1; n <= core_Integral::gauss::GetPointsMax(); ++n )
  {
    const int nGaussPts = n;

    double val = 0;
    for ( size_t k = 0; k < U.size() - 1; ++k )
    {
      const double
        gaussVal = core_Integral::gauss::Compute(&F, U[k], U[k+1], nGaussPts);
      //
      val += gaussVal;
    }

    std::cout << "interval gaussVal [" << nGaussPts << "] = " << val << std::endl;
  }

  // TODO: NYI

  return res.success();
}
