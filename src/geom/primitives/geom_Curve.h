//-----------------------------------------------------------------------------
// Created on: 05 August 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef geom_Curve_HeaderFile
#define geom_Curve_HeaderFile

// Geometry includes
#include <mobius/geom_Geometry.h>

// Core includes
#include <mobius/core_XYZ.h>

namespace mobius {

//! Base class for 3D parametric curves.
class geom_Curve : public geom_Geometry
{
// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_Curve( const core_IsoTransformChain& tChain = core_IsoTransformChain() );

  mobiusGeom_EXPORT virtual
    ~geom_Curve();

public:

  virtual double
    MinParameter() const = 0;

  virtual double
    MaxParameter() const = 0;

  virtual void
    Eval(const double u,
         xyz&         P) const = 0;

};

//! Convenience shortcut.
typedef geom_Curve curve;

};

#endif
