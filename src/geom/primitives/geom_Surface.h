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

#ifndef geom_Surface_HeaderFile
#define geom_Surface_HeaderFile

// Core includes
#include <mobius/core_Ptr.h>

// Geometry includes
#include <mobius/geom_Geometry.h>
#include <mobius/geom_Point.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
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
