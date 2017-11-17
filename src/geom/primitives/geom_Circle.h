//-----------------------------------------------------------------------------
// Created on: 05 September 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef geom_Circle_HeaderFile
#define geom_Circle_HeaderFile

// Geometry includes
#include <mobius/geom_Curve.h>

namespace mobius {

//! Parametric spatial circle.
class geom_Circle : public geom_Curve
{
// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_Circle( const double                  radius,
                 const core_IsoTransformChain& tChain = core_IsoTransformChain() );

  mobiusGeom_EXPORT virtual
    ~geom_Circle();

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
    Eval(const double u,
         xyz&         P) const;

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

  //! Radius of the circle.
  double m_fRadius;

};

};

#endif
