//-----------------------------------------------------------------------------
// Created on: 03 March 2018
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

#ifndef geom_FairBCurveCoeff_HeaderFile
#define geom_FairBCurveCoeff_HeaderFile

// Geom includes
#include <mobius/geom.h>

// Core includes
#include <mobius/core_UnivariateFunc.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Base class for curve fairing coefficients.
class geom_FairBCurveCoeff : public core_UnivariateFunc
{
public:

  //! ctor.
  //! \param[in] lambda fairing coefficient.
  geom_FairBCurveCoeff(const double lambda) : core_UnivariateFunc()
  {
    m_fLambda = lambda;
  }

public:

  //! \return fairing coefficient.
  double GetLambda() const
  {
    return m_fLambda;
  }

  //! Sets fairing coefficient.
  //! \param[in] lambda fairing coefficient.
  void SetLambda(const double lambda)
  {
    m_fLambda = lambda;
  }

protected:

  double m_fLambda; //!< Fairing coefficient.

};

}

#endif
