//-----------------------------------------------------------------------------
// Created on: 08 October 2015
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

#ifndef geom_PolyLine_HeaderFile
#define geom_PolyLine_HeaderFile

// Geometry includes
#include <mobius/geom_Link.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Non-parametric polyline.
class geom_PolyLine : public geom_Geometry
{
// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_PolyLine();

  mobiusGeom_EXPORT virtual
    ~geom_PolyLine();

public:

  mobiusGeom_EXPORT virtual void
    Bounds(double& xMin, double& xMax,
           double& yMin, double& yMax,
           double& zMin, double& zMax) const;

public:

  mobiusGeom_EXPORT void
    AddLink(const Ptr<geom_Link>& link);

  mobiusGeom_EXPORT int
    NumLinks() const;

  mobiusGeom_EXPORT const std::vector< Ptr<geom_Link> >&
    Links() const;

  mobiusGeom_EXPORT const Ptr<geom_Link>&
    Link(const size_t idx) const;

private:

  std::vector< Ptr<geom_Link> > m_links; //!< Links.

};

};

#endif
