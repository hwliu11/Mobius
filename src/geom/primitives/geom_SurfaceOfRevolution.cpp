//-----------------------------------------------------------------------------
// Created on: 10 January 2023
//-----------------------------------------------------------------------------
// Copyright (c) 2023-present, Sergey Slyadnev
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
#include <mobius/geom_SurfaceOfRevolution.h>

// Standard includes
#include <math.h>

//-----------------------------------------------------------------------------

mobius::geom_SurfaceOfRevolution::geom_SurfaceOfRevolution(const t_ptr<t_curve>& curve,
                                                           const t_axis&         axis)
: geom_Surface (),
  m_c          (curve),
  m_ax         (axis)
{
}

//-----------------------------------------------------------------------------

mobius::geom_SurfaceOfRevolution::~geom_SurfaceOfRevolution()
{}

//-----------------------------------------------------------------------------

void mobius::geom_SurfaceOfRevolution::GetBounds(double& xMin, double& xMax,
                                                 double& yMin, double& yMax,
                                                 double& zMin, double& zMax) const
{
  // TODO: NYI

  // c(u)
  double c_x_min, c_x_max, c_y_min, c_y_max, c_z_min, c_z_max;
  m_c->GetBounds(c_x_min, c_x_max, c_y_min, c_y_max, c_z_min, c_z_max);

  // Set results
  xMin = c_x_min;
  xMax = c_x_max;
  yMin = c_y_min;
  yMax = c_y_max;
  zMin = c_z_min;
  zMax = c_z_max;
}

//-----------------------------------------------------------------------------

double mobius::geom_SurfaceOfRevolution::GetMinParameter_U() const
{
  return 0.;
}

//-----------------------------------------------------------------------------

double mobius::geom_SurfaceOfRevolution::GetMaxParameter_U() const
{
  return 2*M_PI;
}

//-----------------------------------------------------------------------------

double mobius::geom_SurfaceOfRevolution::GetMinParameter_V() const
{
  return m_c->GetMinParameter();
}

//-----------------------------------------------------------------------------

double mobius::geom_SurfaceOfRevolution::GetMaxParameter_V() const
{
  return m_c->GetMaxParameter();
}

//-----------------------------------------------------------------------------

void mobius::geom_SurfaceOfRevolution::Eval(const double u,
                                            const double v,
                                            t_xyz&       S) const
{
  t_xyz P;
  m_c->Eval(v, P);

  core_Quaternion qn(m_ax.GetDirection(), u);
  double mx[3][3];
  qn.AsMatrix3x3(mx);

  P.Transform(mx);
  S = P;
}
