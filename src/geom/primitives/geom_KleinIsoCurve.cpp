//-----------------------------------------------------------------------------
// Created on: 16 December 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

// Own include
#include <mobius/geom_KleinIsoCurve.h>

// Geometry includes
#include <mobius/geom_KleinBottle.h>

// Standard includes
#include <math.h>

//! Constructor.
//! \param r     [in] radius of the hole.
//! \param type  [in] type of iso (U or V).
//! \param param [in] parameter value to freeze.
mobius::geom_KleinIsoCurve::geom_KleinIsoCurve(const double  r,
                                               const IsoType type,
                                               const double  param)
: geom_Curve(), m_fR(r), m_type(type), m_fParam(param)
{
}

//! Destructor.
mobius::geom_KleinIsoCurve::~geom_KleinIsoCurve()
{}

//! Calculates boundary box for the Klein isoparametric curve.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::geom_KleinIsoCurve::Bounds(double& xMin, double& xMax,
                                        double& yMin, double& yMax,
                                        double& zMin, double& zMax) const
{
  xMin = -m_fR; xMax = m_fR;
  yMin = -m_fR; yMax = m_fR;
  zMin = -m_fR; zMax = m_fR;
}

//! \return min parameter.
double mobius::geom_KleinIsoCurve::MinParameter() const
{
  return 0.0;
}

//! \return max parameter.
double mobius::geom_KleinIsoCurve::MaxParameter() const
{
  return 2*M_PI;
}

//! Evaluates Klein iso-curve.
//! \param p [in] parameter value to evaluate curve for.
//! \param C [out] 3D point evaluated for the given parameter.
void mobius::geom_KleinIsoCurve::Eval(const double p,
                                      xyz&         C) const
{
  double u = ( (m_type == Iso_U) ? m_fParam : p );
  double v = ( (m_type == Iso_V) ? m_fParam : p );

  // Evaluate surface with fixed parameter
  double x, y, z;
  geom_KleinBottle::Eval(m_fR, u, v, x, y, z);

  // Set output parameter
  C.SetX(x);
  C.SetY(y);
  C.SetZ(z);
}
