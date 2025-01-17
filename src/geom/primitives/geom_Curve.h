//-----------------------------------------------------------------------------
// Created on: 05 August 2013
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

#ifndef geom_Curve_HeaderFile
#define geom_Curve_HeaderFile

// Geometry includes
#include <mobius/geom_Geometry.h>

// Core includes
#include <mobius/core_XYZ.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Base class for 3D parametric curves.
class geom_Curve : public geom_Geometry
{
// Construction & destruction:
public:

  //! Constructor accepting transformations to apply.
  //! \param[in] tChain associated transformations.
  mobiusGeom_EXPORT
    geom_Curve( const core_IsoTransformChain& tChain = core_IsoTransformChain() );

  //! Destructor.
  mobiusGeom_EXPORT virtual
    ~geom_Curve();

public:

  virtual double
    GetMinParameter() const = 0;

  virtual double
    GetMaxParameter() const = 0;

  virtual void
    Eval(const double t,
         t_xyz&       C) const = 0;

  virtual void
    Eval_D1(const double t,
            t_xyz&       dC_dt) const = 0;

public:

  mobiusGeom_EXPORT t_xyz
    D0(const double t) const;

  mobiusGeom_EXPORT t_xyz
    D1(const double t) const;

};

//! Convenience shortcut.
typedef geom_Curve t_curve;

}

#endif
