//-----------------------------------------------------------------------------
// Created on: 10 June 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef geom_BezierOnRailsSurface_HeaderFile
#define geom_BezierOnRailsSurface_HeaderFile

// Geometry includes
#include <mobius/geom_Curve.h>
#include <mobius/geom_Surface.h>

// BSpl includes
#include <mobius/bspl_ScalarLaw.h>

namespace mobius {

//! Bezier surface constructed on three rail curves. This surface is
//! skinned onto a couple of rail curves named r(u) and q(u). One additional
//! curve c(u) is used as a magnifier for surface points. c(u) is called a
//! middle curve, however, you are free in choosing its position.
//!
//! The main requirement for this surface is to be initialized with compatible
//! curves, so as their degree and parameterization have be the same.
class geom_BezierOnRailsSurface : public geom_Surface
{
// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_BezierOnRailsSurface(const Ptr<curve>&          r,
                              const Ptr<curve>&          c,
                              const Ptr<curve>&          q,
                              const Ptr<bspl_ScalarLaw>& w);

  mobiusGeom_EXPORT virtual
    ~geom_BezierOnRailsSurface();

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
         core_XYZ&    S) const;

// Internal methods:
protected:

  mobiusGeom_EXPORT void
    eval_S1(const double u,
            const double v,
            core_XYZ&    P) const;

  mobiusGeom_EXPORT void
    eval_S2(const double u,
            const double v,
            double&      val) const;

private:

  Ptr<curve>          m_r; //!< First rail curve.
  Ptr<curve>          m_c; //!< Middle curve.
  Ptr<curve>          m_q; //!< Second rail curve.
  Ptr<bspl_ScalarLaw> m_w; //!< Weight law.

};

};

#endif
