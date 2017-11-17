//-----------------------------------------------------------------------------
// Created on: 14 June 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

// Own include
#include <mobius/bspl_NDiscrete.h>

// bspl includes
#include <mobius/bspl_N.h>

//! Constructor.
//! \param U   [in] knot vector.
//! \param deg [in] degree of B-spline basis function being discretized.
mobius::bspl_NDiscrete::bspl_NDiscrete(const std::vector<double>& U,
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
                                     const double   delta,
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
const std::vector<double>& mobius::bspl_NDiscrete::Abscissa() const
{
  return m_abscissa;
}

//! Returns discretized function values.
//! \return function values.
const std::vector<double>& mobius::bspl_NDiscrete::Values() const
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
                                                    const double delta)
{
  const double uMin = m_U[0];
  const double uMax = m_U[m_U.size() - 1];

  double uNext = uMin;
  bspl_N N;
  do
  {
    if ( uNext + delta > uMax )
      uNext = uMax; // Just to slip to the end point (no matter how, but
                    // it should not be missed)

    const double val = N(uNext, m_U, m_iDegree, idx);
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
                                                 const double /*delta*/)
{
  // TODO: NYI
}
