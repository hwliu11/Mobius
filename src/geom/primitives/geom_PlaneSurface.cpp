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

// BSpl includes
#include <mobius/bspl_KnotsUniform.h>

// Core includes
#include <mobius/core_Precision.h>

//-----------------------------------------------------------------------------

mobius::geom_PlaneSurface::geom_PlaneSurface()
//
: geom_Surface ( ),
  m_fUMin      ( -core_Precision::Infinity() ),
  m_fUMax      (  core_Precision::Infinity() ),
  m_fVMin      ( -core_Precision::Infinity() ),
  m_fVMax      (  core_Precision::Infinity() ),
  m_D1         ( 1., 0., 0. ),
  m_D2         ( 0., 1., 0. )
{}

//-----------------------------------------------------------------------------

mobius::geom_PlaneSurface::geom_PlaneSurface(const t_xyz& O,
                                             const t_xyz& D1,
                                             const t_xyz& D2)
//
: geom_Surface ( ),
  m_fUMin      ( -core_Precision::Infinity() ),
  m_fUMax      (  core_Precision::Infinity() ),
  m_fVMin      ( -core_Precision::Infinity() ),
  m_fVMax      (  core_Precision::Infinity() ),
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
  t_xyz min_pt, max_pt;
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
                                     t_xyz&       S) const
{
  S = m_origin + m_D1*u + m_D2*v;
}

//-----------------------------------------------------------------------------

mobius::t_ptr<mobius::geom_Line>
  mobius::geom_PlaneSurface::Iso_U(const double u) const
{
  t_xyz isoOrigin;
  this->Eval(u, 0., isoOrigin);

  t_ptr<geom_Line> line = new geom_Line(isoOrigin, m_D2);

  return line;
}

//-----------------------------------------------------------------------------

mobius::t_ptr<mobius::geom_Line>
  mobius::geom_PlaneSurface::Iso_V(const double v) const
{
  t_xyz isoOrigin;
  this->Eval(0., v, isoOrigin);

  t_ptr<geom_Line> line = new geom_Line(isoOrigin, m_D1);

  return line;
}

//-----------------------------------------------------------------------------

mobius::t_ptr<mobius::t_bsurf>
  mobius::geom_PlaneSurface::ToBSurface(const int degU, const int degV) const
{
  if ( core_Precision::IsInfinite(m_fUMin) || core_Precision::IsInfinite(m_fUMax) ||
       core_Precision::IsInfinite(m_fVMin) || core_Precision::IsInfinite(m_fVMax) )
    return NULL; // Cannot convert infinite surface.

  /* We agree that there are 2 poles in each direction. */

  int r, s;
  std::vector<double> U, V;

  // Prepare U knots.
  if ( bspl_KnotsUniform::Calculate(1, degU, r, U) != bspl_KnotsUniform::ErrCode_NoError )
    return NULL;

  // Prepare V knots.
  if ( bspl_KnotsUniform::Calculate(1, degV, s, V) != bspl_KnotsUniform::ErrCode_NoError )
    return NULL;

  // Evaluate poles.
  t_xyz P00, P01, P10, P11;
  //
  this->Eval(m_fUMin, m_fVMin, P00);
  this->Eval(m_fUMin, m_fVMax, P01);
  this->Eval(m_fUMax, m_fVMin, P10);
  this->Eval(m_fUMax, m_fVMax, P11);

  std::vector< std::vector<t_xyz> > poles = { {P00, P01},
                                              {P10, P11} };

  // Construct B-surface.
  t_ptr<t_bsurf> res = new t_bsurf(poles, U, V, degU, degV);
  //
  return res;
}
