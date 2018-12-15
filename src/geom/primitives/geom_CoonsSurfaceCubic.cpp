//-----------------------------------------------------------------------------
// Created on: 15 December 2018
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
#include <mobius/geom_CoonsSurfaceCubic.h>

// BSpl includes
#include <mobius/bspl_HermiteLaw.h>

//-----------------------------------------------------------------------------

mobius::geom_CoonsSurfaceCubic::geom_CoonsSurfaceCubic(const ptr<curve>& Su0,
                                                       const ptr<curve>& Su1,
                                                       const ptr<curve>& S0v,
                                                       const ptr<curve>& S1v,
                                                       const xyz&        S00,
                                                       const xyz&        S01,
                                                       const xyz&        S10,
                                                       const xyz&        S11,
                                                       const xyz&        dS_du00,
                                                       const xyz&        dS_du01,
                                                       const xyz&        dS_du10,
                                                       const xyz&        dS_du11,
                                                       const xyz&        dS_dv00,
                                                       const xyz&        dS_dv01,
                                                       const xyz&        dS_dv10,
                                                       const xyz&        dS_dv11,
                                                       const xyz&        d2S_dudv00,
                                                       const xyz&        d2S_dudv01,
                                                       const xyz&        d2S_dudv10,
                                                       const xyz&        d2S_dudv11)
: geom_Surface (),
  m_Su0        (Su0),
  m_Su1        (Su1),
  m_S0v        (S0v),
  m_S1v        (S1v),
  m_S00        (S00),
  m_S01        (S01),
  m_S10        (S10),
  m_S11        (S11),
  m_dS_du00    (dS_du00),
  m_dS_du01    (dS_du01),
  m_dS_du10    (dS_du10),
  m_dS_du11    (dS_du11),
  m_dS_dv00    (dS_dv00),
  m_dS_dv01    (dS_dv01),
  m_dS_dv10    (dS_dv10),
  m_dS_dv11    (dS_dv11),
  m_d2S_dudv00 (d2S_dudv00),
  m_d2S_dudv01 (d2S_dudv01),
  m_d2S_dudv10 (d2S_dudv10),
  m_d2S_dudv11 (d2S_dudv11)
{
  this->SetEvalComponent(EvalComponent_All);
}

//-----------------------------------------------------------------------------

mobius::geom_CoonsSurfaceCubic::~geom_CoonsSurfaceCubic()
{}

//-----------------------------------------------------------------------------

void mobius::geom_CoonsSurfaceCubic::GetBounds(double& xMin, double& xMax,
                                               double& yMin, double& yMax,
                                               double& zMin, double& zMax) const
{
  /* =======================================
   *  This approach is very approximate (!)
   * ======================================= */

  // c0(u)
  double c0_x_min, c0_x_max, c0_y_min, c0_y_max, c0_z_min, c0_z_max;
  m_Su0->GetBounds(c0_x_min, c0_x_max, c0_y_min, c0_y_max, c0_z_min, c0_z_max);

  // c1(u)
  double c1_x_min, c1_x_max, c1_y_min, c1_y_max, c1_z_min, c1_z_max;
  m_Su1->GetBounds(c1_x_min, c1_x_max, c1_y_min, c1_y_max, c1_z_min, c1_z_max);

  // b0(v)
  double b0_x_min, b0_x_max, b0_y_min, b0_y_max, b0_z_min, b0_z_max;
  m_S0v->GetBounds(b0_x_min, b0_x_max, b0_y_min, b0_y_max, b0_z_min, b0_z_max);

  // b1(v)
  double b1_x_min, b1_x_max, b1_y_min, b1_y_max, b1_z_min, b1_z_max;
  m_S1v->GetBounds(b1_x_min, b1_x_max, b1_y_min, b1_y_max, b1_z_min, b1_z_max);

  // Set results
  xMin = fmin(c0_x_min, fmin(c1_x_min, fmin(b0_x_min, b1_x_min)));
  xMax = fmax(c0_x_max, fmin(c1_x_max, fmin(b0_x_max, b1_x_max)));
  yMin = fmin(c0_y_min, fmin(c1_y_min, fmin(b0_y_min, b1_y_min)));
  yMax = fmax(c0_y_max, fmin(c1_y_max, fmin(b0_y_max, b1_y_max)));
  zMin = fmin(c0_z_min, fmin(c1_z_min, fmin(b0_z_min, b1_z_min)));
  zMax = fmax(c0_z_max, fmin(c1_z_max, fmin(b0_z_max, b1_z_max)));
}

//-----------------------------------------------------------------------------

double mobius::geom_CoonsSurfaceCubic::GetMinParameter_U() const
{
  return m_Su0->GetMinParameter();
}

//-----------------------------------------------------------------------------

double mobius::geom_CoonsSurfaceCubic::GetMaxParameter_U() const
{
  return m_Su0->GetMaxParameter();
}

//-----------------------------------------------------------------------------

double mobius::geom_CoonsSurfaceCubic::GetMinParameter_V() const
{
  return m_S0v->GetMinParameter();
}

//-----------------------------------------------------------------------------

double mobius::geom_CoonsSurfaceCubic::GetMaxParameter_V() const
{
  return m_S0v->GetMaxParameter();
}

//-----------------------------------------------------------------------------

void mobius::geom_CoonsSurfaceCubic::Eval(const double u,
                                          const double v,
                                          xyz&         S) const
{
  // Compute Boolean sum.
  xyz P1S, P2S, P12S, PS;
  this->Eval_P1S  (u, v, P1S);
  this->Eval_P2S  (u, v, P2S);
  this->Eval_P12S (u, v, P12S);
  //
  PS = P1S + P2S - P12S;

  if ( m_comp == EvalComponent_P1S )
    S = P1S;
  else if ( m_comp == EvalComponent_P2S )
    S = P2S;
  else if ( m_comp == EvalComponent_P12S )
    S = P12S;
  else // All components of the Boolean sum.
    S = PS;
}

//-----------------------------------------------------------------------------

void mobius::geom_CoonsSurfaceCubic::Eval_P1S(const double u,
                                              const double v,
                                              xyz&         S) const
{
  bspl_HermiteLaw alpha0(0);
  bspl_HermiteLaw alpha1(1);
  bspl_HermiteLaw beta0(2);
  bspl_HermiteLaw beta1(3);

  const double a0 = alpha0.Eval(v);
  const double a1 = alpha1.Eval(v);
  const double b0 = beta0.Eval(v);
  const double b1 = beta1.Eval(v);

  S = a0*m_Su0->Eval(u)
    + a1*m_Su1->Eval(u)
    + b0*m_dS_dv00
    + b1*m_dS_dv01;
}

//-----------------------------------------------------------------------------

void mobius::geom_CoonsSurfaceCubic::Eval_P2S(const double u,
                                              const double v,
                                              xyz&         S) const
{
  bspl_HermiteLaw alpha0(0);
  bspl_HermiteLaw alpha1(1);
  bspl_HermiteLaw beta0(2);
  bspl_HermiteLaw beta1(3);

  const double a0 = alpha0.Eval(u);
  const double a1 = alpha1.Eval(u);
  const double b0 = beta0.Eval(u);
  const double b1 = beta1.Eval(u);

  S = a0*m_S0v->Eval(v)
    + a1*m_S1v->Eval(v)
    + b0*m_dS_du01
    + b1*m_dS_du10;
}

//-----------------------------------------------------------------------------

void mobius::geom_CoonsSurfaceCubic::Eval_P12S(const double u,
                                               const double v,
                                               xyz&         S) const
{
  bspl_HermiteLaw alpha0(0);
  bspl_HermiteLaw alpha1(1);
  bspl_HermiteLaw beta0(2);
  bspl_HermiteLaw beta1(3);

  const double a0u = alpha0.Eval(u);
  const double a1u = alpha1.Eval(u);
  const double b0u = beta0.Eval(u);
  const double b1u = beta1.Eval(u);
  const double a0v = alpha0.Eval(v);
  const double a1v = alpha1.Eval(v);
  const double b0v = beta0.Eval(v);
  const double b1v = beta1.Eval(v);

  const xyz row0 = m_S00    *a0v + m_S01    *a1v + m_dS_dv00   *b0v + m_dS_dv01   *b1v;
  const xyz row1 = m_S10    *a0v + m_S11    *a1v + m_dS_dv10   *b0v + m_dS_dv11   *b1v;
  const xyz row2 = m_dS_du00*a0v + m_dS_du01*a1v + m_d2S_dudv00*b0v + m_d2S_dudv01*b1v;
  const xyz row3 = m_dS_du10*a0v + m_dS_du11*a1v + m_d2S_dudv10*b0v + m_d2S_dudv11*b1v;
  //
  const xyz S1 = row0*a0u;
  const xyz S2 = row1*a1u;
  const xyz S3 = row2*b0u;
  const xyz S4 = row3*b1u;

  S = S1 + S2 + S3 + S4;
}
