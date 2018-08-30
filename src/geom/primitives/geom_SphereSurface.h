//-----------------------------------------------------------------------------
// Created on: 05 September 2014
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

#ifndef geom_SphereSurface_HeaderFile
#define geom_SphereSurface_HeaderFile

// Geometry includes
#include <mobius/geom_Circle.h>
#include <mobius/geom_Surface.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
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
    geom_SphereSurface( const adouble                  radius,
                        const core_IsoTransformChain& tChain = core_IsoTransformChain() );

  mobiusGeom_EXPORT virtual
    ~geom_SphereSurface();

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
         xyz&         S) const;

public:

  mobiusGeom_EXPORT ptr<geom_Circle>
    Iso_U(const adouble u) const;

  mobiusGeom_EXPORT ptr<geom_Circle>
    Iso_V(const adouble v) const;

public:

  //! Accessor for the radius.
  //! \return radius of the circle.
  adouble Radius() const
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

  adouble m_fRadius; //!< Radius of the sphere.

};

};

#endif
