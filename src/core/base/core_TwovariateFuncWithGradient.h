//-----------------------------------------------------------------------------
// Created on: 18 May 2019
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

#ifndef core_TwovariateFuncWithGradient_HeaderFile
#define core_TwovariateFuncWithGradient_HeaderFile

// core includes
#include <mobius/core_TwovariateFunc.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Abstract twovariate scalar function with gradient.
class core_TwovariateFuncWithGradient : public core_TwovariateFunc
{
public:

  //! Evaluates function with its gradient for the given argument value.
  //! \param[in]  x     first argument.
  //! \param[in]  y     second argument.
  //! \param[out] F     function value.
  //! \param[out] dF_dX partial first-order derivative by the first argument.
  //! \param[out] dF_dY partial first-order derivative by the second argument.
  virtual void
    EvalWithGrad(const double x,
                 const double y,
                 double&      F,
                 double&      dF_dX,
                 double&      dF_dY) const = 0;

};

}

#endif
