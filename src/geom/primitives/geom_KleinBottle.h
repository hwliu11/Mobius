//-----------------------------------------------------------------------------
// Created on: 15 December 2014
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

#ifndef geom_KleinBottle_HeaderFile
#define geom_KleinBottle_HeaderFile

// Geometry includes
#include <mobius/geom_KleinIsoCurve.h>
#include <mobius/geom_Surface.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
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
    GetBounds(double& xMin, double& xMax,
              double& yMin, double& yMax,
              double& zMin, double& zMax) const override;

public:

  mobiusGeom_EXPORT virtual double
    GetMinParameter_U() const override;

  mobiusGeom_EXPORT virtual double
    GetMaxParameter_U() const override;

  mobiusGeom_EXPORT virtual double
    GetMinParameter_V() const override;

  mobiusGeom_EXPORT virtual double
    GetMaxParameter_V() const override;

  mobiusGeom_EXPORT virtual void
    Eval(const double u,
         const double v,
         t_xyz&       C) const override;

public:

  mobiusGeom_EXPORT t_ptr<geom_KleinIsoCurve>
    Iso_U(const double u) const;

  mobiusGeom_EXPORT t_ptr<geom_KleinIsoCurve>
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

}

#endif
