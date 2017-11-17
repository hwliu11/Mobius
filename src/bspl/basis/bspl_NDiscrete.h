//-----------------------------------------------------------------------------
// Created on: 14 June 2013
// Created by: Sergey SLYADNEV
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
    bspl_NDiscrete(const std::vector<double>& U,
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
            const double   delta,
            const Strategy strategy = Strategy_UniformAbscissa);

  mobiusBSpl_EXPORT const std::vector<double>&
    Abscissa() const;

  mobiusBSpl_EXPORT const std::vector<double>&
    Values() const;

  mobiusBSpl_EXPORT bool
    IsDone() const;

private:

  void
    performUniformAbscissa(const int    idx,
                           const double delta);

  void
    performUniformChord(const int    idx,
                        const double delta);

private:

  //! Knot vector.
  std::vector<double> m_U;

  //! Degree of B-spline basis functions.
  int m_iDegree;

  //! Abscissa.
  std::vector<double> m_abscissa;

  //! Function values.
  std::vector<double> m_values;

  //! Indicates whether the algorithm has completed its calculations or not.
  bool m_bIsDone;

};

};

#endif
