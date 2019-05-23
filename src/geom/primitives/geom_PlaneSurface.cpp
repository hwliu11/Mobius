//-----------------------------------------------------------------------------
// Created on: 23 May 2019
//-----------------------------------------------------------------------------
// Copyright (c) 2019-present, Sergey Slyadnev
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
#include <mobius/geom_PlaneSurface.h>

//-----------------------------------------------------------------------------

mobius::geom_PlaneSurface::geom_PlaneSurface()
//
: geom_Surface ( ),
  m_fUMin      ( -DBL_MAX ),
  m_fUMax      (  DBL_MAX ),
  m_fVMin      ( -DBL_MAX ),
  m_fVMax      (  DBL_MAX ),
  m_D1         ( 1., 0., 0. ),
  m_D2         ( 0., 1., 0. )
{}

//-----------------------------------------------------------------------------

mobius::geom_PlaneSurface::geom_PlaneSurface(const xyz& O,
                                             const xyz& D1,
                                             const xyz& D2)
//
: geom_Surface ( ),
  m_fUMin      ( -DBL_MAX ),
  m_fUMax      (  DBL_MAX ),
  m_fVMin      ( -DBL_MAX ),
  m_fVMax      (  DBL_MAX ),
  m_origin     ( O ),
  m_D1         ( D1 ),
  m_D2         ( D2 )
{}

//-----------------------------------------------------------------------------

mobius::geom_PlaneSurface::~geom_PlaneSurface()
{}

//-----------------------------------------------------------------------------

void mobius::geom_PlaneSurface::GetBounds(double& xMin, double& xMax,
                                          double& yMin, double& yMax,
                                          double& zMin, double& zMax) const
{
  xyz min_pt, max_pt;
  //
  this->Eval(m_fUMin, m_fVMin, min_pt);
  this->Eval(m_fUMax, m_fVMax, max_pt);

  xMin = min_pt.X();
  xMax = max_pt.X();
  yMin = min_pt.Y();
  yMax = max_pt.Y();
  zMin = min_pt.Z();
  zMax = max_pt.Z();
}

//-----------------------------------------------------------------------------

double mobius::geom_PlaneSurface::GetMinParameter_U() const
{
  return m_fUMin;
}

//-----------------------------------------------------------------------------

double mobius::geom_PlaneSurface::GetMaxParameter_U() const
{
  return m_fUMax;
}

//-----------------------------------------------------------------------------

double mobius::geom_PlaneSurface::GetMinParameter_V() const
{
  return m_fVMin;
}

//-----------------------------------------------------------------------------

double mobius::geom_PlaneSurface::GetMaxParameter_V() const
{
  return m_fVMax;
}

//-----------------------------------------------------------------------------

void mobius::geom_PlaneSurface::Eval(const double u,
                                     const double v,
                                     xyz&         S) const
{
  S = m_origin + m_D1*u + m_D2*v;
}

//-----------------------------------------------------------------------------

mobius::ptr<mobius::geom_Line>
  mobius::geom_PlaneSurface::Iso_U(const double u) const
{
  xyz isoOrigin;
  this->Eval(u, 0., isoOrigin);

  ptr<geom_Line> line = new geom_Line(isoOrigin, m_D2);

  return line;
}

//-----------------------------------------------------------------------------

mobius::ptr<mobius::geom_Line>
  mobius::geom_PlaneSurface::Iso_V(const double v) const
{
  xyz isoOrigin;
  this->Eval(0., v, isoOrigin);

  ptr<geom_Line> line = new geom_Line(isoOrigin, m_D1);

  return line;
}
