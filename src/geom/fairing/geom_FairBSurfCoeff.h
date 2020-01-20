//-----------------------------------------------------------------------------
// Created on: 20 August 2018
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

#ifndef geom_FairBSurfCoeff_HeaderFile
#define geom_FairBSurfCoeff_HeaderFile

// Geom includes
#include <mobius/geom.h>

// Core includes
#include <mobius/core_Integral.h>
#include <mobius/core_TwovariateFunc.h>

// Standard includes
#include <algorithm>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Base class for surface fairing coefficients.
class geom_FairBSurfCoeff : public core_TwovariateFunc
{
public:

  //! ctor.
  //! \param[in] lambda fairing coefficient.
  geom_FairBSurfCoeff(const double lambda) : core_TwovariateFunc()
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
  double SetLambda(const double lambda)
  {
    m_fLambda = lambda;
  }

public:

  virtual void GetSupportSpans(int& ifirst, int& ilast,
                               int& jfirst, int& jlast) const = 0;

public:

  //! Convenience function to integrate by knot spans.
  double Integral(const std::vector<double>& U,
                  const std::vector<double>& V,
                  const int                  p,
                  const int                  q)
  {
    const int NUM_GAUSS_PT_U = std::max(p, 3);
    const int NUM_GAUSS_PT_V = std::max(q, 3);

    // According to the local support property of B-spline basis functions
    // (see for example P2.1 at p. 55 in "The NURBS Book"), not all spans
    // are effective.
    int iFirst = 0, iLast = int( U.size() - 1 ), // Global range.
        jFirst = 0, jLast = int( V.size() - 1 ); // Global range.
    //
    this->GetSupportSpans(iFirst, iLast, jFirst, jLast);

    // Integrate in each span individually for better accuracy.
    double result = 0;
    for ( size_t i = iFirst; i < iLast; ++i )
    {
      if ( U[i] == U[i+1] ) continue; // Skip multiple knots.

      for ( size_t j = jFirst; j < jLast; ++j )
      {
        if ( V[j] == V[j+1] ) continue; // Skip multiple knots.

        // Gauss integration in each knot span.
        const double
          gaussVal = core_Integral::gauss::Compute(this,
                                                   U[i], U[i+1],
                                                   V[j], V[j+1],
                                                   NUM_GAUSS_PT_U, NUM_GAUSS_PT_V);
        //
        result += gaussVal;
      }
    }

    return result;
  }

protected:

  double m_fLambda; //!< Fairing coefficient.

};

};

#endif
