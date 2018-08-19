//-----------------------------------------------------------------------------
// Created on: 25 July 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2018, Sergey Slyadnev
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

#ifndef core_Integral_HeaderFile
#define core_Integral_HeaderFile

// Core includes
#include <mobius/core_Ptr.h>
#include <mobius/core_TwovariateFunc.h>
#include <mobius/core_UnivariateFunc.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Utilities for numerical integration.
namespace core_Integral
{
  //! Integrates the passed function by midpoint (rectangle) rule. This
  //! function is the worst one in terms of efficiency, though it is very
  //! simple and clear. We use rectangle rule mostly to validate other
  //! quadrature methods like Gauss-Legendre.
  //!
  //! \param[in]  F        univariate function in question.
  //! \param[in]  a        lower bound.
  //! \param[in]  b        upper bound.
  //! \param[in]  n        number of bins.
  //! \param[out] numEvals number of function evaluations.
  //! \return integral value.
  mobiusCore_EXPORT double
    ComputeRect(core_UnivariateFunc* F,
                const double         a,
                const double         b,
                const int            n,
                int&                 numEvals);

  //! Integrates the passed function by midpoint (rectangle) rule without
  //! returning the number of function evaluation requests.
  //!
  //! \param[in] F univariate function in question.
  //! \param[in] a lower bound.
  //! \param[in] b upper bound.
  //! \param[in] n number of bins.
  //! \return integral value.
  mobiusCore_EXPORT double
    ComputeRect(core_UnivariateFunc* F,
                const double         a,
                const double         b,
                const int            n);

  //! Gaussian quadratures.
  namespace gauss {

    //! \return Max allowed number of Gauss-Legendre points.
    mobiusCore_EXPORT int
      GetPointsMax();

    //! Returns Gauss-Legendre points for the given order of integration <n>.
    //! \param[in]  n      order of integration.
    //! \param[out] points Gauss-Legendre points.
    mobiusCore_EXPORT void
      GetPoints(const int n, std::vector<double>& points);

    //! Returns Gauss-Legendre weights for the given order of integration <n>.
    //! \param[in]  n       order of integration.
    //! \param[out] weights Gauss-Legendre weights.
    mobiusCore_EXPORT void
      GetWeights(const int n, std::vector<double>& weights);

    //! Returns the integral of the function <F> between <a> and <b>, by
    //! <n>-point Gauss-Legendre integration: the function is evaluated
    //! exactly <n> times at interior points in the range of integration.
    //!
    //! For more details see
    //!
    //! [Press, William H.; Teukolsky, Saul A.; Vetterling, William T.; Flannery, Brian P. (2007).
    //!  Numerical Recipes: The Art of Scientific Computing (3rd ed.).
    //!  New York: Cambridge University Press. ISBN 978-0-521-88068-8.]
    //!
    //! \param[in]  F        univariate function in question.
    //! \param[in]  a        lower bound.
    //! \param[in]  b        upper bound.
    //! \param[in]  n        number of integration points.
    //! \param[out] numEvals number of function evaluations.
    //! \return computed integral value.
    mobiusCore_EXPORT double
      Compute(core_UnivariateFunc* F,
              const double         a,
              const double         b,
              const int            n,
              int&                 numEvals);

    //! Returns the integral of the function <F> without returning the number
    //! of evaluation requests.
    //!
    //! \param[in] F univariate function in question.
    //! \param[in] a lower bound.
    //! \param[in] b upper bound.
    //! \param[in] n number of integration points.
    //! \return computed integral value.
    mobiusCore_EXPORT double
      Compute(core_UnivariateFunc* F,
              const double         a,
              const double         b,
              const int            n);

    //! Returns the integral of the passed two-variate function <F> defined
    //! in a simple rectangular domain.
    //!
    //! \param[in] F      function to integrate.
    //! \param[in] x0     left bound of X variable.
    //! \param[in] x1     right bound of X variable.
    //! \param[in] y0     left bound of Y variable.
    //! \param[in] y1     right bound of Z variable.
    //! \param[in] orderX order for Gauss integration along X variable.
    //! \param[in] orderY order for Gauss integration along Y variable.
    //! \return computed integral value.
    mobiusCore_EXPORT double
      Compute(core_TwovariateFunc* F,
              const double         x0,
              const double         x1,
              const double         y0,
              const double         y1,
              const int            orderX,
              const int            orderY);

  };

};

};

#endif
