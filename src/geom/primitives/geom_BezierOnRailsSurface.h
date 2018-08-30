//-----------------------------------------------------------------------------
// Created on: 10 June 2013
//-----------------------------------------------------------------------------
// Copyright (c) 2017, Sergey Slyadnev
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//    * Neither the name of Sergey Slyadnev nor the
//      names of all contributors may be used to endorse or promote products
//      derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//-----------------------------------------------------------------------------

#ifndef geom_BezierOnRailsSurface_HeaderFile
#define geom_BezierOnRailsSurface_HeaderFile

// Geometry includes
#include <mobius/geom_Curve.h>
#include <mobius/geom_Surface.h>

// BSpl includes
#include <mobius/bspl_ScalarLaw.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
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
    geom_BezierOnRailsSurface(const ptr<curve>&          r,
                              const ptr<curve>&          c,
                              const ptr<curve>&          q,
                              const ptr<bspl_ScalarLaw>& w);

  mobiusGeom_EXPORT virtual
    ~geom_BezierOnRailsSurface();

// Interface methods:
public:

  mobiusGeom_EXPORT virtual void
    Bounds(adouble& xMin, adouble& xMax,
           adouble& yMin, adouble& yMax,
           adouble& zMin, adouble& zMax) const;

  mobiusGeom_EXPORT virtual adouble
    MinParameter_U() const;

  mobiusGeom_EXPORT virtual adouble
    MaxParameter_U() const;

  mobiusGeom_EXPORT virtual adouble
    MinParameter_V() const;

  mobiusGeom_EXPORT virtual adouble
    MaxParameter_V() const;

  mobiusGeom_EXPORT virtual void
    Eval(const adouble u,
         const adouble v,
         core_XYZ&    S) const;

// Internal methods:
protected:

  mobiusGeom_EXPORT void
    eval_S1(const adouble u,
            const adouble v,
            core_XYZ&    P) const;

  mobiusGeom_EXPORT void
    eval_S2(const adouble u,
            const adouble v,
            adouble&      val) const;

private:

  ptr<curve>          m_r; //!< First rail curve.
  ptr<curve>          m_c; //!< Middle curve.
  ptr<curve>          m_q; //!< Second rail curve.
  ptr<bspl_ScalarLaw> m_w; //!< Weight law.

};

};

#endif
