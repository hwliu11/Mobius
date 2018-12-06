//-----------------------------------------------------------------------------
// Created on: 03 December 2018
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
#include <mobius/geom_CoonsSurface.h>

//-----------------------------------------------------------------------------

mobius::geom_CoonsSurface::geom_CoonsSurface(const ptr<curve>& c0,
                                             const ptr<curve>& c1,
                                             const ptr<curve>& b0,
                                             const ptr<curve>& b1,
                                             const xyz&        p00,
                                             const xyz&        p01,
                                             const xyz&        p10,
                                             const xyz&        p11)
: geom_Surface (),
  m_c0         (c0),
  m_c1         (c1),
  m_b0         (b0),
  m_b1         (b1),
  m_p00        (p00),
  m_p01        (p01),
  m_p10        (p10),
  m_p11        (p11)
{
}

//-----------------------------------------------------------------------------

mobius::geom_CoonsSurface::~geom_CoonsSurface()
{}

//-----------------------------------------------------------------------------

void mobius::geom_CoonsSurface::Bounds(double& xMin, double& xMax,
                                       double& yMin, double& yMax,
                                       double& zMin, double& zMax) const
{
  /* =======================================
   *  This approach is very approximate (!)
   * ======================================= */

  // c0(u)
  double c0_x_min, c0_x_max, c0_y_min, c0_y_max, c0_z_min, c0_z_max;
  m_c0->Bounds(c0_x_min, c0_x_max, c0_y_min, c0_y_max, c0_z_min, c0_z_max);

  // c1(u)
  double c1_x_min, c1_x_max, c1_y_min, c1_y_max, c1_z_min, c1_z_max;
  m_c1->Bounds(c1_x_min, c1_x_max, c1_y_min, c1_y_max, c1_z_min, c1_z_max);

  // b0(u)
  double b0_x_min, b0_x_max, b0_y_min, b0_y_max, b0_z_min, b0_z_max;
  m_b0->Bounds(b0_x_min, b0_x_max, b0_y_min, b0_y_max, b0_z_min, b0_z_max);

  // b1(u)
  double b1_x_min, b1_x_max, b1_y_min, b1_y_max, b1_z_min, b1_z_max;
  m_b1->Bounds(b1_x_min, b1_x_max, b1_y_min, b1_y_max, b1_z_min, b1_z_max);

  // Set results
  xMin = fmin(c0_x_min, fmin(c1_x_min, fmin(b0_x_min, b1_x_min)));
  xMax = fmax(c0_x_max, fmin(c1_x_max, fmin(b0_x_max, b1_x_max)));
  yMin = fmin(c0_y_min, fmin(c1_y_min, fmin(b0_y_min, b1_y_min)));
  yMax = fmax(c0_y_max, fmin(c1_y_max, fmin(b0_y_max, b1_y_max)));
  zMin = fmin(c0_z_min, fmin(c1_z_min, fmin(b0_z_min, b1_z_min)));
  zMax = fmax(c0_z_max, fmin(c1_z_max, fmin(b0_z_max, b1_z_max)));
}

//-----------------------------------------------------------------------------

double mobius::geom_CoonsSurface::MinParameter_U() const
{
  return m_c0->MinParameter();
}

//-----------------------------------------------------------------------------

double mobius::geom_CoonsSurface::MaxParameter_U() const
{
  return m_c0->MaxParameter();
}

//-----------------------------------------------------------------------------

double mobius::geom_CoonsSurface::MinParameter_V() const
{
  return m_b0->MinParameter();
}

//-----------------------------------------------------------------------------

double mobius::geom_CoonsSurface::MaxParameter_V() const
{
  return m_b0->MaxParameter();
}

//-----------------------------------------------------------------------------

void mobius::geom_CoonsSurface::Eval(const double u,
                                     const double v,
                                     xyz&         S) const
{
  xyz c0, c1, b0, b1;
  m_c0->Eval(u, c0);
  m_c1->Eval(u, c1);
  m_b0->Eval(v, b0);
  m_b1->Eval(v, b1);

  S = (1-v)*c0 + v*c1 + (1-u)*b0 + u*b1
    - (1-u)*(1-v)*m_p00 - u*(1-v)*m_p10 - (1-u)*v*m_p01 - u*v*m_p11;
}
