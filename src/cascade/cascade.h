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

#ifndef cascade_HeaderFile
#define cascade_HeaderFile

#if defined _WIN32
  #if defined mobiusCascade_EXPORTS
    #define mobiusCascade_EXPORT __declspec(dllexport)
  #else
    #define mobiusCascade_EXPORT __declspec(dllimport)
  #endif
#else
  #define mobiusCascade_EXPORT
#endif

#define cascade_NotUsed(x) x

// Core includes
#include <mobius/core_Ptr.h>
#include <mobius/core_XYZ.h>

// Geom includes
#include <mobius/geom_BSplineCurve.h>
#include <mobius/geom_BSplineSurface.h>

// OpenCascade includes
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <gp_Pnt.hxx>

//-----------------------------------------------------------------------------
// DOXY group definition
//-----------------------------------------------------------------------------
//! \defgroup MOBIUS_CASCADE OpenCascade
//!
//! Connectors with OpenCascade kernel.
//-----------------------------------------------------------------------------

namespace mobius
{
  //! \ingroup MOBIUS_CASCADE
  //!
  //! Common conversion utilities.
  class cascade
  {
  public:

    //! Converts Mobius 3D point to OpenCascade 3D point.
    //! \param[in] coords point to convert.
    //! \return converted point.
    static gp_Pnt GetOpenCascadePnt(const xyz& coords)
    {
      return gp_Pnt( coords.X(), coords.Y(), coords.Z() );
    }

    //! Converts OpenCascade 3D point to Mobius 3D point.
    //! \param[in] coords point to convert.
    //! \return converted point.
    static xyz GetMobiusPnt(const gp_Pnt& coords)
    {
      return xyz( coords.X(), coords.Y(), coords.Z() );
    }

    //! Converts Mobius B-curve to OpenCascade B-curve.
    //! \param[in] curve Mobius curve to convert.
    //! \return OpenCascade curve.
    mobiusCascade_EXPORT static Handle(Geom_BSplineCurve)
      GetOpenCascadeBCurve(const ptr<bcurve>& curve);

    //! Converts OpenCascade B-curve to Mobius B-curve.
    //! \param[in] curve OpenCascade curve to convert.
    //! \return Mobius curve.
    mobiusCascade_EXPORT static ptr<bcurve>
      GetMobiusBCurve(const Handle(Geom_BSplineCurve)& curve);

    //! Converts Mobius B-surface to OpenCascade B-surface.
    //! \param[in] surface Mobius surface to convert.
    //! \return OpenCascade surface.
    mobiusCascade_EXPORT static Handle(Geom_BSplineSurface)
      GetOpenCascadeBSurface(const ptr<bsurf>& surface);

    //! Converts OpenCascade B-surface to Mobius B-surface.
    //! \param[in] surface OpenCascade surface to convert.
    //! \return Mobius surface.
    mobiusCascade_EXPORT static ptr<bsurf>
      GetMobiusBSurface(const Handle(Geom_BSplineSurface)& surface);

  };
};

#endif
