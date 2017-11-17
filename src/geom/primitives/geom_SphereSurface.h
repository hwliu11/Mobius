//-----------------------------------------------------------------------------
// Created on: 05 September 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef geom_SphereSurface_HeaderFile
#define geom_SphereSurface_HeaderFile

// Geometry includes
#include <mobius/geom_Circle.h>
#include <mobius/geom_Surface.h>

namespace mobius {

//! Spherical surface having the following parameterization:
//! <pre>
//!    x = x0 + r cos(theta) cos(phi);
//!    y = y0 + r cos(theta) sin(phi);
//!    z = z0 + r sin(theta);
//! </pre>
//! By convention:
//! <pre>
//!   theta -- v
//!   phi -- u
//! </pre>
//! Both parameters are angles in range [0;2PI].
class geom_SphereSurface : public geom_Surface
{
// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_SphereSurface( const double                  radius,
                        const core_IsoTransformChain& tChain = core_IsoTransformChain() );

  mobiusGeom_EXPORT virtual
    ~geom_SphereSurface();

// Interface methods:
public:

  mobiusGeom_EXPORT virtual void
    Bounds(double& xMin, double& xMax,
           double& yMin, double& yMax,
           double& zMin, double& zMax) const;

  mobiusGeom_EXPORT virtual double
    MinParameter_U() const;

  mobiusGeom_EXPORT virtual double
    MaxParameter_U() const;

  mobiusGeom_EXPORT virtual double
    MinParameter_V() const;

  mobiusGeom_EXPORT virtual double
    MaxParameter_V() const;

  mobiusGeom_EXPORT virtual void
    Eval(const double u,
         const double v,
         xyz&         S) const;

public:

  mobiusGeom_EXPORT Ptr<geom_Circle>
    Iso_U(const double u) const;

  mobiusGeom_EXPORT Ptr<geom_Circle>
    Iso_V(const double v) const;

public:

  //! Accessor for the radius.
  //! \return radius of the circle.
  double Radius() const
  {
    return m_fRadius;
  }

  //! Accessor for the center.
  //! \return center of the circle.
  xyz Center() const
  {
    return m_tChain.Apply( xyz::O() );
  }

private:

  double m_fRadius; //!< Radius of the sphere.

};

};

#endif
