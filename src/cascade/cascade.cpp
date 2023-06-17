//-----------------------------------------------------------------------------
// Created on: 04 December 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2018-present, Sergey Slyadnev
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

// Own include
#include <mobius/cascade.h>

// Cascade includes
#include <mobius/cascade_BSplineCurve.h>
#include <mobius/cascade_BSplineSurface.h>

// OpenCascade includes
#include <GeomConvert.hxx>

using namespace mobius;

//-----------------------------------------------------------------------------

Handle(Geom_BSplineCurve)
  cascade::GetOpenCascadeBCurve(const t_ptr<t_bcurve>& curve)
{
  cascade_BSplineCurve tool(curve);
  tool.DirectConvert();

  return tool.GetOpenCascadeCurve();
}

//-----------------------------------------------------------------------------

t_ptr<t_bcurve>
  cascade::GetMobiusBCurve(const Handle(Geom_BSplineCurve)& curve)
{
  cascade_BSplineCurve tool(curve);
  tool.DirectConvert();

  return tool.GetMobiusCurve();
}

//-----------------------------------------------------------------------------

Handle(Geom_BSplineSurface)
  cascade::GetOpenCascadeBSurface(const t_ptr<t_bsurf>& surface)
{
  cascade_BSplineSurface tool(surface);
  tool.DirectConvert();

  return tool.GetOpenCascadeSurface();
}

//-----------------------------------------------------------------------------

t_ptr<t_bsurf>
  cascade::GetMobiusBSurface(const Handle(Geom_BSplineSurface)& surface)
{
  cascade_BSplineSurface tool(surface);
  tool.DirectConvert();

  return tool.GetMobiusSurface();
}

//-----------------------------------------------------------------------------

Handle(Geom_Plane)
  cascade::GetOpenCascadePlane(const t_ptr<t_plane>& surface)
{
  gp_Pnt O  = GetOpenCascadePnt( surface->GetOrigin() );
  gp_Vec Du = GetOpenCascadeVec( surface->GetD1() );
  gp_Vec Dv = GetOpenCascadeVec( surface->GetD2() );
  //
  return new Geom_Plane( gp_Ax3(O, Du^Dv, Du) );
}

//-----------------------------------------------------------------------------

t_ptr<t_plane>
  cascade::GetMobiusPlane(const Handle(Geom_Plane)& surface)
{
  const gp_Ax3& ax3 = surface->Position();
  //
  t_xyz O  = GetMobiusPnt( ax3.Location() );
  t_xyz Du = GetMobiusVec( ax3.XDirection() );
  t_xyz Dv = GetMobiusVec( ax3.YDirection() );

  return new t_plane(O, Du, Dv);
}

//-----------------------------------------------------------------------------

Handle(Geom_SurfaceOfRevolution)
  cascade::GetOpenCascadeRevolSurf(const t_ptr<t_surfRevol>& surface)
{
  // Convert the meridian curve.
  Handle(Geom_BSplineCurve)
    occC = GetOpenCascadeBCurve( t_ptr<t_bcurve>::DownCast( surface->GetMeridian() ) );

  // Convert the axis.
  const t_axis& ax = surface->GetAxis();
  gp_Ax1        occAx( GetOpenCascadePnt( ax.GetPosition() ),
                       GetOpenCascadeXYZ( ax.GetDirection() ) );

  Handle(Geom_SurfaceOfRevolution)
    res = new Geom_SurfaceOfRevolution(occC, occAx);

  return res;
}

//-----------------------------------------------------------------------------

t_ptr<t_surfRevol>
  cascade::GetMobiusRevolSurf(const Handle(Geom_SurfaceOfRevolution)& surface)
{
  // Check generatrix: it should be a spline.
  Handle(Geom_BSplineCurve)
    occGeneratrix = GeomConvert::CurveToBSplineCurve( surface->BasisCurve() );
  //
  if ( occGeneratrix.IsNull() )
    return nullptr;

  // Convert the generatrix curve.
  t_ptr<t_bcurve> mbGeneratrix = GetMobiusBCurve(occGeneratrix);

  // Convert axis.
  t_xyz mbPos = GetMobiusPnt( surface->Axis().Location() );
  t_xyz mbVec = GetMobiusVec( surface->Axis().Direction() );
  //
  t_axis mbAxis(mbPos, mbVec);

  // Construct a surface of revolution.
  t_ptr<t_surfRevol> result = new t_surfRevol(mbGeneratrix, mbAxis);

  return result;
}
