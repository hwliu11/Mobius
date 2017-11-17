//-----------------------------------------------------------------------------
// Created on: 16 December 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef geom_KleinIsoCurve_HeaderFile
#define geom_KleinIsoCurve_HeaderFile

// Geometry includes
#include <mobius/geom_Curve.h>

namespace mobius {

//! Isoparametric curve for Klein bottle.
class geom_KleinIsoCurve : public geom_Curve
{
public:

  //! Which kind of iso (U or V).
  enum IsoType
  {
    Iso_U,
    Iso_V
  };

// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_KleinIsoCurve(const double  r,
                       const IsoType type,
                       const double  param);

  mobiusGeom_EXPORT virtual
    ~geom_KleinIsoCurve();

public:

  mobiusGeom_EXPORT virtual void
    Bounds(double& xMin, double& xMax,
           double& yMin, double& yMax,
           double& zMin, double& zMax) const;

public:

  mobiusGeom_EXPORT virtual double
    MinParameter() const;

  mobiusGeom_EXPORT virtual double
    MaxParameter() const;

  mobiusGeom_EXPORT virtual void
    Eval(const double p,
         xyz&         C) const;

private:

  //! Radius of hole.
  double m_fR;

  //! Type of iso.
  IsoType m_type;

  //! Fixed parameter value.
  double m_fParam;

};

};

#endif
