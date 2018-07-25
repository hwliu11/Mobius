//-----------------------------------------------------------------------------
// Created on: 24 December 2014
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

#ifndef cascade_BSplineSurface_HeaderFile
#define cascade_BSplineSurface_HeaderFile

// Cascade includes
#include <mobius/cascade.h>

// Geom includes
#include <mobius/geom_BSplineSurface.h>

// OCCT includes
#include <Geom_BSplineSurface.hxx>

namespace mobius {

//! Bridge for conversions between Mobius and OCCT B-surfaces.
class cascade_BSplineSurface
{
public:

  mobiusCascade_EXPORT
    cascade_BSplineSurface(const ptr<bsurf>& mobiusSurface);

  mobiusCascade_EXPORT
    cascade_BSplineSurface(const Handle(Geom_BSplineSurface)& occtSurface);

  mobiusCascade_EXPORT
    ~cascade_BSplineSurface();

public:

  mobiusCascade_EXPORT void
    DirectConvert();

public:

  mobiusCascade_EXPORT const ptr<bsurf>&
    GetMobiusSurface() const;

  mobiusCascade_EXPORT const Handle(Geom_BSplineSurface)&
    GetOpenCascadeSurface() const;

  mobiusCascade_EXPORT bool
    IsDone() const;

protected:

  mobiusCascade_EXPORT void
    convertToOpenCascade();

  mobiusCascade_EXPORT void
    convertToMobius();

private:

  //! Mobius surface.
  ptr<bsurf> m_mobiusSurface;

  //! OCCT surface.
  Handle(Geom_BSplineSurface) m_occtSurface;

  //! Indicates whether conversion is done or not.
  bool m_bIsDone;


};

};

#endif
