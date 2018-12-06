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

//-----------------------------------------------------------------------------

Handle(Geom_BSplineCurve)
  mobius::cascade::GetOpenCascadeBCurve(const ptr<bcurve>& curve)
{
  cascade_BSplineCurve tool(curve);
  tool.DirectConvert();

  return tool.GetOpenCascadeCurve();
}

//-----------------------------------------------------------------------------

mobius::ptr<mobius::bcurve>
  mobius::cascade::GetMobiusBCurve(const Handle(Geom_BSplineCurve)& curve)
{
  cascade_BSplineCurve tool(curve);
  tool.DirectConvert();

  return tool.GetMobiusCurve();
}

//-----------------------------------------------------------------------------

Handle(Geom_BSplineSurface)
  mobius::cascade::GetOpenCascadeBSurface(const ptr<bsurf>& surface)
{
  cascade_BSplineSurface tool(surface);
  tool.DirectConvert();

  return tool.GetOpenCascadeSurface();
}

//-----------------------------------------------------------------------------

mobius::ptr<mobius::bsurf>
  mobius::cascade::GetMobiusBSurface(const Handle(Geom_BSplineSurface)& surface)
{
  cascade_BSplineSurface tool(surface);
  tool.DirectConvert();

  return tool.GetMobiusSurface();
}
