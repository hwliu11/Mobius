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
#include <mobius/geom_SphereSurface.h>

// STD includes
#include <math.h>

//! Constructs surface by origin point and radius.
//! \param radius [in] radius of the sphere.
//! \param tChain [in] transformation chain to apply.
mobius::geom_SphereSurface::geom_SphereSurface(const double                  radius,
                                               const core_IsoTransformChain& tChain)
: geom_Surface(tChain),
  m_fRadius(radius)
{
}

//! Destructor.
mobius::geom_SphereSurface::~geom_SphereSurface()
{}

//! Calculates boundary box for the surface.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::geom_SphereSurface::Bounds(double& xMin, double& xMax,
                                        double& yMin, double& yMax,
                                        double& zMin, double& zMax) const
{
  xyz global_center = m_tChain.Apply( this->Center() );
  xyz rad_pt(m_fRadius, m_fRadius, m_fRadius);

  xyz min_pt = global_center - rad_pt;
  xyz max_pt = global_center + rad_pt;

  xMin = min_pt.X();
  xMax = max_pt.X();
  yMin = min_pt.Y();
  yMax = max_pt.Y();
  zMin = min_pt.Z();
  zMax = max_pt.Z();
}

//! Returns minimal U parameter.
//! \return parameter value.
double mobius::geom_SphereSurface::MinParameter_U() const
{
  return 0.0;
}

//! Returns maximal U parameter.
//! \return parameter value.
double mobius::geom_SphereSurface::MaxParameter_U() const
{
  return 2*M_PI;
}

//! Returns minimal V parameter.
//! \return parameter value.
double mobius::geom_SphereSurface::MinParameter_V() const
{
  return -M_PI;
}

//! Returns maximal V parameter.
//! \return parameter value.
double mobius::geom_SphereSurface::MaxParameter_V() const
{
  return M_PI;
}

//! Evaluates surface in the given parametric point (u, v).
//! \param u [in]  first parameter.
//! \param v [in]  second parameter.
//! \param P [out] evaluated spatial point S(u, v).
void mobius::geom_SphereSurface::Eval(const double u,
                                      const double v,
                                      xyz&         S) const
{
  /* ==========================
   *  Sphere point in local CS
   * ========================== */

  const double x = m_fRadius*cos(v)*cos(u);
  const double y = m_fRadius*cos(v)*sin(u);
  const double z = m_fRadius*sin(v);

  S.SetX(x);
  S.SetY(y);
  S.SetZ(z);

  /* =====================
   *  Switch to global CS
   * ===================== */

  S = m_tChain.Apply(S);
}

//! Returns isoparametric curve for a fixed {u}.
//! \param u [in] fixed parameter.
//! \return isoparametric curve.
mobius::Ptr<mobius::geom_Circle>
  mobius::geom_SphereSurface::Iso_U(const double u) const
{
  const double r             = m_fRadius; // Radius is the same
  const double ang_around_OX = M_PI/2.0;  // Flip circle vertical
  const double ang_around_OZ = u;         // Rotate to have iso-u

  // Rotation to represent iso-u in a local system of axes of sphere
  core_Quaternion qn_around_OX(xyz::OX(), ang_around_OX);
  core_Quaternion qn_around_OZ(xyz::OZ(), ang_around_OZ);
  core_Quaternion qn_local = qn_around_OZ * qn_around_OX;

  // Prepare transformation chain for the resulting isoline
  core_IsoTransformChain tChain;
  tChain << m_tChain << core_IsoTransform( qn_local, xyz() );

  // Return result
  return new geom_Circle(r, tChain);
}

//! Returns isoparametric curve for a fixed {v}.
//! \param v [in] fixed parameter.
//! \return isoparametric curve.
mobius::Ptr<mobius::geom_Circle>
  mobius::geom_SphereSurface::Iso_V(const double v) const
{
  const double r  = m_fRadius*cos(v); // Radius is the same
  const double dZ = m_fRadius*sin(v); // Elevation

  // Transformation to apply in local CS of sphere
  xyz elevation(0.0, 0.0, dZ);
  core_IsoTransform T_local(core_Quaternion(), elevation);

  // Prepare transformation chain for the resulting isoline
  core_IsoTransformChain tChain;
  tChain << m_tChain << T_local;

  // Return result
  return new geom_Circle(r, tChain);
}
