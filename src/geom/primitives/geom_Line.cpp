//-----------------------------------------------------------------------------
// Created on: 22 May 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

// Own include
#include <mobius/geom_Line.h>

//! Constructor.
//! \param origin [in] origin of line.
//! \param dir    [in] direction vector for line.
mobius::geom_Line::geom_Line(const xyz& origin,
                             const xyz& dir)
: geom_Curve(),
  m_origin(origin),
  m_dir( dir.Normalized() )
{
}

//! Destructor.
mobius::geom_Line::~geom_Line()
{}

//! Calculates boundary box for the line.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::geom_Line::Bounds(double& xMin, double& xMax,
                               double& yMin, double& yMax,
                               double& zMin, double& zMax) const
{
  // Even though it is possible to reduce infinite space to something more
  // representative for line, we do not do it as we do not have any
  // practical need for that
  xMin = -DBL_MAX;
  yMin = -DBL_MAX;
  zMin = -DBL_MAX;
  xMax =  DBL_MAX;
  yMax =  DBL_MAX;
  zMax =  DBL_MAX;
}

//! Returns minimal parameter value.
//! \return minimal parameter value.
double mobius::geom_Line::MinParameter() const
{
  return -DBL_MAX;
}

//! Returns maximal parameter value.
//! \return maximal parameter value.
double mobius::geom_Line::MaxParameter() const
{
  return DBL_MAX;
}

//! Evaluates line for the given parameter.
//! \param u [in]  parameter value to evaluate curve for.
//! \param P [out] 3D point corresponding to the given parameter on curve.
void mobius::geom_Line::Eval(const double u,
                             xyz&         P) const
{
  P = m_origin + m_dir*u;
}
