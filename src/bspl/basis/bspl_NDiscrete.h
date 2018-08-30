//-----------------------------------------------------------------------------
// Created on: 14 June 2013
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

#ifndef bspl_NDiscrete_HeaderFile
#define bspl_NDiscrete_HeaderFile

// bspl includes
#include <mobius/bspl.h>

namespace mobius {

//! \ingroup MOBIUS_BSPL
//!
//! Algorithm performing discretization of B-spline basis function.
//!
//! \todo complete description
class bspl_NDiscrete
{
// Construction & destruction:
public:

  mobiusBSpl_EXPORT
    bspl_NDiscrete(const std::vector<adouble>& U,
                   const int                  deg);

public:

  //! Discretization policy.
  enum Strategy
  {
    //! Uniform distribution of U values. This is the simplest discretization
    //! approach. It obviously gives poor concentration of points on highly
    //! curved segments.
    Strategy_UniformAbscissa,

    //! Uniform chord length. This strategy allows to achieve better quality
    //! on highly curved segments.
    Strategy_UniformChord
  };

public:

  mobiusBSpl_EXPORT void
    Perform(const int      idx,
            const adouble   delta,
            const Strategy strategy = Strategy_UniformAbscissa);

  mobiusBSpl_EXPORT const std::vector<adouble>&
    Abscissa() const;

  mobiusBSpl_EXPORT const std::vector<adouble>&
    Values() const;

  mobiusBSpl_EXPORT bool
    IsDone() const;

private:

  void
    performUniformAbscissa(const int    idx,
                           const adouble delta);

  void
    performUniformChord(const int    idx,
                        const adouble delta);

private:

  //! Knot vector.
  std::vector<adouble> m_U;

  //! Degree of B-spline basis functions.
  int m_iDegree;

  //! Abscissa.
  std::vector<adouble> m_abscissa;

  //! Function values.
  std::vector<adouble> m_values;

  //! Indicates whether the algorithm has completed its calculations or not.
  bool m_bIsDone;

};

};

#endif
