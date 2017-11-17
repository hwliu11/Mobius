//-----------------------------------------------------------------------------
// Created on: 10 June 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef geom_Surface_HeaderFile
#define geom_Surface_HeaderFile

// Core includes
#include <mobius/core_Ptr.h>

// Geometry includes
#include <mobius/geom_Geometry.h>
#include <mobius/geom_Point.h>

namespace mobius {

//! Base class for surfaces. Declares the common interface methods.
//! All surfaces are actually evaluators for points and derivatives.
//! It is possible to integrate them into any environment designed to query
//! surface data.
class geom_Surface : public geom_Geometry
{
// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_Surface( const core_IsoTransformChain& tChain = core_IsoTransformChain() );

  mobiusGeom_EXPORT
    virtual ~geom_Surface();

// Interface methods:
public:

  virtual double
    MinParameter_U() const = 0;

  virtual double
    MaxParameter_U() const = 0;

  virtual double
    MinParameter_V() const = 0;

  virtual double
    MaxParameter_V() const = 0;

  virtual void
    Eval(const double u,
         const double v,
         xyz&         S) const = 0;

};

};

#endif
