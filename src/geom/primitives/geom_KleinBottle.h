//-----------------------------------------------------------------------------
// Created on: 15 December 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef geom_KleinBottle_HeaderFile
#define geom_KleinBottle_HeaderFile

// Geometry includes
#include <mobius/geom_KleinIsoCurve.h>
#include <mobius/geom_Surface.h>

namespace mobius {

//! Klein bottle.
//!
//! \todo provide more comments.
class geom_KleinBottle : public geom_Surface
{
// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_KleinBottle(const double r);

  mobiusGeom_EXPORT virtual
    ~geom_KleinBottle();

public:

  mobiusGeom_EXPORT virtual void
    Bounds(double& xMin, double& xMax,
           double& yMin, double& yMax,
           double& zMin, double& zMax) const;

public:

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
         xyz&         C) const;

public:

  mobiusGeom_EXPORT Ptr<geom_KleinIsoCurve>
    Iso_U(const double u) const;

  mobiusGeom_EXPORT Ptr<geom_KleinIsoCurve>
    Iso_V(const double v) const;

public:

  mobiusGeom_EXPORT static void
    Eval(const double r,
         const double u,
         const double v,
         double&      x,
         double&      y,
         double&      z);

private:

  //! Radius of hole.
  double m_fR;

};

};

#endif
