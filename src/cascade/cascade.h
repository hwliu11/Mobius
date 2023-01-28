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
#include <mobius/geom_PlaneSurface.h>
#include <mobius/geom_SurfaceOfRevolution.h>

// Poly includes
#include <mobius/poly_Mesh.h>

// OpenCascade includes
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_SurfaceOfRevolution.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <Poly_Triangulation.hxx>

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

    //! Converts Mobius 3D point to OpenCascade 3D XYZ tuple.
    //! \param[in] coords point to convert.
    //! \return converted point.
    static gp_XYZ GetOpenCascadeXYZ(const t_xyz& coords)
    {
      return gp_XYZ( coords.X(), coords.Y(), coords.Z() );
    }

    //! Converts Mobius 3D point to OpenCascade 3D point.
    //! \param[in] coords point to convert.
    //! \return converted point.
    static gp_Pnt GetOpenCascadePnt(const t_xyz& coords)
    {
      return gp_Pnt( coords.X(), coords.Y(), coords.Z() );
    }

    //! Converts Mobius 2D point to OpenCascade 2D point.
    //! \param[in] coords point to convert.
    //! \return converted point.
    static gp_Pnt2d GetOpenCascadePnt2d(const t_uv& coords)
    {
      return gp_Pnt2d( coords.U(), coords.V() );
    }

    //! Converts Mobius 3D point to OpenCascade 3D vector.
    //! \param[in] coords point to convert.
    //! \return converted vector.
    static gp_Vec GetOpenCascadeVec(const t_xyz& coords)
    {
      return gp_Vec( coords.X(), coords.Y(), coords.Z() );
    }

    //! Converts OpenCascade 3D point to Mobius 3D point.
    //! \param[in] coords point to convert.
    //! \return converted point.
    static t_xyz GetMobiusPnt(const gp_Pnt& coords)
    {
      return t_xyz( coords.X(), coords.Y(), coords.Z() );
    }

    //! Converts OpenCascade 3D point to Mobius 3D point.
    //! \param[in] coords point to convert.
    //! \return converted point.
    static t_xyz GetMobiusPnt(const gp_XYZ& coords)
    {
      return t_xyz( coords.X(), coords.Y(), coords.Z() );
    }

    //! Converts OpenCascade 3D vector to Mobius 3D vector.
    //! \param[in] coords vector to convert.
    //! \return converted triple of coordinates.
    static t_xyz GetMobiusVec(const gp_Vec& coords)
    {
      return t_xyz( coords.X(), coords.Y(), coords.Z() );
    }

    //! Converts OpenCascade 2D point to Mobius 2D point.
    //! \param[in] coords point to convert.
    //! \return converted point.
    static t_uv GetMobiusPnt2d(const gp_Pnt2d& coords)
    {
      return t_uv( coords.X(), coords.Y() );
    }

    //! Converts OpenCascade 2D point to Mobius 2D point.
    //! \param[in] coords point to convert.
    //! \return converted point.
    static t_uv GetMobiusPnt2d(const gp_XY& coords)
    {
      return t_uv( coords.X(), coords.Y() );
    }

    //! Converts Mobius B-curve to OpenCascade B-curve.
    //! \param[in] curve Mobius curve to convert.
    //! \return OpenCascade curve.
    mobiusCascade_EXPORT static Handle(Geom_BSplineCurve)
      GetOpenCascadeBCurve(const t_ptr<t_bcurve>& curve);

    //! Converts OpenCascade B-curve to Mobius B-curve.
    //! \param[in] curve OpenCascade curve to convert.
    //! \return Mobius curve.
    mobiusCascade_EXPORT static t_ptr<t_bcurve>
      GetMobiusBCurve(const Handle(Geom_BSplineCurve)& curve);

    //! Converts Mobius B-surface to OpenCascade B-surface.
    //! \param[in] surface Mobius surface to convert.
    //! \return OpenCascade surface.
    mobiusCascade_EXPORT static Handle(Geom_BSplineSurface)
      GetOpenCascadeBSurface(const t_ptr<t_bsurf>& surface);

    //! Converts OpenCascade B-surface to Mobius B-surface.
    //! \param[in] surface OpenCascade surface to convert.
    //! \return Mobius surface.
    mobiusCascade_EXPORT static t_ptr<t_bsurf>
      GetMobiusBSurface(const Handle(Geom_BSplineSurface)& surface);

    //! Converts Mobius plane to OpenCascade plane.
    //! \param[in] surface Mobius surface to convert.
    //! \return OpenCascade surface.
    mobiusCascade_EXPORT static Handle(Geom_Plane)
      GetOpenCascadePlane(const t_ptr<t_plane>& surface);

    //! Converts OpenCascade plane to Mobius plane.
    //! \param[in] surface OpenCascade surface to convert.
    //! \return Mobius surface.
    mobiusCascade_EXPORT static t_ptr<t_plane>
      GetMobiusPlane(const Handle(Geom_Plane)& surface);

    //! Converts Mobius mesh to OpenCascade triangulation.
    //! \param[in] mesh Mobius mesh to convert.
    //! \return OpenCascade mesh.
    mobiusCascade_EXPORT static Handle(Poly_Triangulation)
      GetOpenCascadeMesh(const t_ptr<t_mesh>& mesh);

    //! Converts OpenCascade triangulation to Mobius mesh.
    //! \param[in] tris OpenCascade triangulation to convert.
    //! \return Mobius mesh.
    mobiusCascade_EXPORT static t_ptr<t_mesh>
      GetMobiusMesh(const Handle(Poly_Triangulation)& tris);

    //! Converts Mobius surface of revolution to OpenCascade counterpart.
    //! \param[in] surface Mobius surface to convert.
    //! \return OpenCascade surface.
    mobiusCascade_EXPORT static Handle(Geom_SurfaceOfRevolution)
      GetOpenCascadeRevolSurf(const t_ptr<t_surfRevol>& surface);

  };
}

#endif
