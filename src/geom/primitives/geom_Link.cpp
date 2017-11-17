//-----------------------------------------------------------------------------
// Created on: 23 May 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

// Own include
#include <mobius/geom_Link.h>

//! Complete constructor.
//! \param P1 [in] first point.
//! \param P2 [in] second point.
mobius::geom_Link::geom_Link(const xyz& P1,
                             const xyz& P2) : geom_Geometry()
{
  m_p1 = P1;
  m_p2 = P2;
}

//! Destructor.
mobius::geom_Link::~geom_Link()
{}

//! Calculates boundary box for the link.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::geom_Link::Bounds(double& xMin, double& xMax,
                               double& yMin, double& yMax,
                               double& zMin, double& zMax) const
{
  double x_min = DBL_MAX, x_max = -DBL_MAX;
  double y_min = DBL_MAX, y_max = -DBL_MAX;
  double z_min = DBL_MAX, z_max = -DBL_MAX;

  const double x[] = { m_p1.X(), m_p2.X() };
  const double y[] = { m_p1.Y(), m_p2.Y() };
  const double z[] = { m_p1.Z(), m_p2.Z() };

  for ( int c = 0; c < 2; ++c )
  {
    if ( x[c] > x_max )
      x_max = x[c];
    if ( x[c] < x_min )
      x_min = x[c];
    if ( y[c] > y_max )
      y_max = y[c];
    if ( y[c] < y_min )
      y_min = y[c];
    if ( z[c] > z_max )
      z_max = z[c];
    if ( z[c] < z_min )
      z_min = z[c];
  }

  // Set results
  xMin = x_min;
  xMax = x_max;
  yMin = y_min;
  yMax = y_max;
  zMin = z_min;
  zMax = z_max;
}
