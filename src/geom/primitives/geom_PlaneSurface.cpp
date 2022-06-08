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
  m_D1         ( 1., 0., 0. ),
  m_D2         ( 0., 1., 0. ),
  m_fUMin      ( -core_Precision::Infinity() ),
  m_fUMax      (  core_Precision::Infinity() ),
  m_fVMin      ( -core_Precision::Infinity() ),
  m_fVMax      (  core_Precision::Infinity() )
{}

//-----------------------------------------------------------------------------

mobius::geom_PlaneSurface::geom_PlaneSurface(const t_xyz& O,
                                             const t_xyz& D1,
                                             const t_xyz& D2)
//
: geom_Surface ( ),
  m_origin     ( O ),
  m_D1         ( D1 ),
  m_D2         ( D2 ),
  m_fUMin      ( -core_Precision::Infinity() ),
  m_fUMax      (  core_Precision::Infinity() ),
  m_fVMin      ( -core_Precision::Infinity() ),
  m_fVMax      (  core_Precision::Infinity() )
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

void mobius::geom_PlaneSurface::InvertPoint(const t_xyz& P,
                                            t_uv&        params) const
{
  t_xyz        vec = (P - m_origin);
  const double u   = vec.Dot(m_D1);
  const double v   = vec.Dot(m_D2);

  params.SetU(u);
  params.SetV(v);
}

//-----------------------------------------------------------------------------

mobius::t_ptr<mobius::t_bsurf>
  mobius::geom_PlaneSurface::ToBSurface(const int degU, const int degV) const
{
  if ( core_Precision::IsInfinite(m_fUMin) || core_Precision::IsInfinite(m_fUMax) ||
       core_Precision::IsInfinite(m_fVMin) || core_Precision::IsInfinite(m_fVMax) )
    return nullptr; // Cannot convert infinite surface.

  int r, s;
  std::vector<double> U, V;

  // Prepare U knots.
  const int nu = degU;
  if ( bspl_KnotsUniform::Calculate(nu, degU, r, U) != bspl_KnotsUniform::ErrCode_NoError )
    return nullptr;

  // Prepare V knots.
  const int nv = degV;
  if ( bspl_KnotsUniform::Calculate(nv, degV, s, V) != bspl_KnotsUniform::ErrCode_NoError )
    return nullptr;

  // Evaluate (u,v) coordinates for poles.
  std::vector<double> uValues, vValues;
  //
  const double uDelta = (m_fUMax - m_fUMin) / nu;
  const double vDelta = (m_fVMax - m_fVMin) / nv;
  //
  double nextU = m_fUMin, nextV = m_fVMin;
  bool   uStop = false,   vStop = false;
  //
  while ( !uStop )
  {
    if ( fabs(nextU - m_fUMax) < 1.e-4 )
    {
      nextU = m_fUMax;
      uStop = true;
    }
    //
    uValues.push_back(nextU);

    nextU += uDelta;
  }
  //
  while ( !vStop )
  {
    if ( fabs(nextV - m_fVMax) < 1.e-4 )
    {
      nextV = m_fVMax;
      vStop = true;
    }
    //
    vValues.push_back(nextV);

    nextV += vDelta;
  }

  // Evaluate poles.
  std::vector< std::vector<t_xyz> > poles;
  //
  for ( size_t i = 0; i < uValues.size(); ++i )
  {
    std::vector<t_xyz> uIsoPoles;
    for ( size_t j = 0; j < vValues.size(); ++j )
    {
      t_xyz P;
      this->Eval(uValues[i], vValues[j], P);
      //
      uIsoPoles.push_back(P);
    }

    poles.push_back(uIsoPoles);
  }

  // Construct B-surface.
  t_ptr<t_bsurf> res = new t_bsurf(poles, U, V, degU, degV);
  //
  return res;
}

//-----------------------------------------------------------------------------

void mobius::geom_PlaneSurface::TrimByPoints(const t_ptr<t_pcloud>& pts,
                                             const double           enlargePerc)
{
  double uMin =  core_Precision::Infinity();
  double uMax = -core_Precision::Infinity();
  double vMin =  core_Precision::Infinity();
  double vMax = -core_Precision::Infinity();

  // Invert each point.
  for ( int k = 0; k < pts->GetNumberOfPoints(); ++k )
  {
    const t_xyz& P = pts->GetPoint(k);

    // Invert.
    t_uv uv;
    this->InvertPoint(P, uv);

    if ( uv.U() < uMin )
      uMin = uv.U();
    if ( uv.U() > uMax )
      uMax = uv.U();
    if ( uv.V() < vMin )
      vMin = uv.V();
    if ( uv.V() > vMax )
      vMax = uv.V();
  }

  const double uEnlarge = fabs(uMax - uMin)*enlargePerc/100.;
  const double vEnlarge = fabs(vMax - vMin)*enlargePerc/100.;

  // Set limits.
  m_fUMin = uMin - uEnlarge;
  m_fUMax = uMax + uEnlarge;
  m_fVMin = vMin - vEnlarge;
  m_fVMax = vMax + vEnlarge;
}
