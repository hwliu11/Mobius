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

// Own include
#include <mobius/bspl_NDiscrete.h>

// bspl includes
#include <mobius/bspl_N.h>

//! Constructor.
//! \param U   [in] knot vector.
//! \param deg [in] degree of B-spline basis function being discretized.
mobius::bspl_NDiscrete::bspl_NDiscrete(const std::vector<adouble>& U,
                                       const int                  deg)
{
  m_U       = U;
  m_iDegree = deg;
  m_bIsDone = false;
}

//! Performs discretization.
//! \param idx      [in] index of B-spline basis function being discretized.
//! \param delta    [in] discretization delta.
//! \param strategy [in] discretization strategy.
void mobius::bspl_NDiscrete::Perform(const int      idx,
                                     const adouble   delta,
                                     const Strategy strategy)
{
  m_bIsDone = false;
  switch ( strategy )
  {
    case Strategy_UniformAbscissa:
      this->performUniformAbscissa(idx, delta);
      break;
    case Strategy_UniformChord:
      this->performUniformChord(idx, delta);
      break;
    default:
      break;
  }
}

//! Returns discretized abscissa.
//! \return abscissa values.
const std::vector<adouble>& mobius::bspl_NDiscrete::Abscissa() const
{
  return m_abscissa;
}

//! Returns discretized function values.
//! \return function values.
const std::vector<adouble>& mobius::bspl_NDiscrete::Values() const
{
  return m_values;
}

//! Returns true if the algorithm has successfully completed its
//! calculations. False -- otherwise.
//! \return true/false.
bool mobius::bspl_NDiscrete::IsDone() const
{
  return m_bIsDone;
}

//! Performs discretization by uniform distribution of abscissa points.
//! \param idx   [in] index of the basis B-spline function to discretize.
//! \param delta [in] discretization delta.
void mobius::bspl_NDiscrete::performUniformAbscissa(const int    idx,
                                                    const adouble delta)
{
  const adouble uMin = m_U[0];
  const adouble uMax = m_U[m_U.size() - 1];

  adouble uNext = uMin;
  bspl_N N;
  do
  {
    if ( uNext + delta > uMax )
      uNext = uMax; // Just to slip to the end point (no matter how, but
                    // it should not be missed)

    const adouble val = N(uNext, m_U, m_iDegree, idx);
    m_values.push_back(val);
    m_abscissa.push_back(uNext);
    uNext += delta;
  }
  while ( uNext < uMax );

  m_bIsDone = true;
}

//! \todo NYI bspl_NDiscrete::performUniformChord()
//!
//! Performs discretization so as to have uniform chord segments on basis
//! B-spline curves.
//! \param idx   [in] index of the basis B-spline function to discretize.
//! \param delta [in] discretization delta.
void mobius::bspl_NDiscrete::performUniformChord(const int /*idx*/,
                                                 const adouble /*delta*/)
{
  // TODO: NYI
}
