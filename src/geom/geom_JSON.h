//-----------------------------------------------------------------------------
// Created on: 19 June 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2013-present, Sergey Slyadnev
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

#ifndef geom_JSON_HeaderFile
#define geom_JSON_HeaderFile

// Geom includes
#include <mobius/geom_BSplineCurve.h>
#include <mobius/geom_BSplineSurface.h>

// Core includes
#include <mobius/core_JSON.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Utility class to process JSON objects representing geometric primitives.
class geom_JSON : public core_JSON
{
public:

  mobiusGeom_EXPORT
    geom_JSON();

  mobiusGeom_EXPORT
    geom_JSON(const std::string& json);

  mobiusGeom_EXPORT
    ~geom_JSON();

public:

  mobiusGeom_EXPORT void
    DumpBCurve(const t_ptr<t_bcurve>& curve);

  mobiusGeom_EXPORT void
    DumpBSurface(const t_ptr<t_bsurf>& surface);

  mobiusGeom_EXPORT bool
    ExtractBCurve(t_ptr<t_bcurve>& curve) const;

  mobiusGeom_EXPORT bool
    ExtractBSurface(t_ptr<t_bsurf>& surface) const;

};

};

#endif
