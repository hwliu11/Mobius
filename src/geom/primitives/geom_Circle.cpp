//-----------------------------------------------------------------------------
// Created on: 05 September 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

// Own include
#include <mobius/geom_Circle.h>

// Standard includes
#include <math.h>

//! Constructor.
//! \param radius [in] radius of the circle.
//! \param tChain [in] transformation chain to apply.
mobius::geom_Circle::geom_Circle(const double                  radius,
                                 const core_IsoTransformChain& tChain)
: geom_Curve(tChain),
  m_fRadius(radius)
{}

//! Destructor.
mobius::geom_Circle::~geom_Circle()
{}

//! Calculates boundary box for the circle.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::geom_Circle::Bounds(double& xMin, double& xMax,
                                 double& yMin, double& yMax,
                                 double& zMin, double& zMax) const
{
  // TODO: NYI
  xMin = -DBL_MAX;
  yMin = -DBL_MAX;
  zMin = -DBL_MAX;
  xMax =  DBL_MAX;
  yMax =  DBL_MAX;
  zMax =  DBL_MAX;
}

//! Returns minimal parameter value.
//! \return minimal parameter value.
double mobius::geom_Circle::MinParameter() const
{
  return 0.0;
}

//! Returns maximal parameter value.
//! \return maximal parameter value.
double mobius::geom_Circle::MaxParameter() const
{
  return 2*M_PI;
}

//! Evaluates circle for the given parameter.
//! \param u [in] parameter value to evaluate curve for.
//! \param P [out] 3D point corresponding to the given parameter on curve.
void mobius::geom_Circle::Eval(const double u,
                               xyz&         P) const
{
  const double x = m_fRadius*cos(u);
  const double y = m_fRadius*sin(u);
  const double z = 0.0;

  xyz P_local(x, y, z);
  P = m_tChain.Apply(P_local);
}
