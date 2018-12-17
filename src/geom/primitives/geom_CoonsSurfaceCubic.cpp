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
                                                       const ptr<curve>& dSu0_dv,
                                                       const ptr<curve>& dSu1_dv,
                                                       const ptr<curve>& dS0v_du,
                                                       const ptr<curve>& dS1v_du,
                                                       const xyz&        d2S_dudv00,
                                                       const xyz&        d2S_dudv01,
                                                       const xyz&        d2S_dudv10,
                                                       const xyz&        d2S_dudv11)
: geom_Surface (),
  m_Su0        (Su0),
  m_Su1        (Su1),
  m_S0v        (S0v),
  m_S1v        (S1v),
  m_dSu0_dv    (dSu0_dv),
  m_dSu1_dv    (dSu1_dv),
  m_dS0v_du    (dS0v_du),
  m_dS1v_du    (dS1v_du),
  m_d2S_dudv00 (d2S_dudv00),
  m_d2S_dudv01 (d2S_dudv01),
  m_d2S_dudv10 (d2S_dudv10),
  m_d2S_dudv11 (d2S_dudv11)
{
  // Initialize positional constraints.
  m_S00 = m_Su0->Eval(0.0);
  m_S01 = m_S0v->Eval(1.0);
  m_S10 = m_Su0->Eval(1.0);
  m_S11 = m_S1v->Eval(1.0);

  // Initialize derivative values at corner points.
  m_dS00_du = m_dS0v_du->Eval(0.0);
  m_dS01_du = m_dS0v_du->Eval(1.0);
  m_dS10_du = m_dS1v_du->Eval(0.0);
  m_dS11_du = m_dS1v_du->Eval(1.0);
  //
  m_dS00_dv = m_dSu0_dv->Eval(0.0);
  m_dS01_dv = m_dSu1_dv->Eval(0.0);
  m_dS10_dv = m_dSu0_dv->Eval(1.0);
  m_dS11_dv = m_dSu1_dv->Eval(1.0);

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
  //
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

void mobius::geom_CoonsSurfaceCubic::Eval_D1(const double u,
                                             const double v,
                                             xyz&         dU,
                                             xyz&         dV) const
{
  // Compute derivative of the Boolean sum.
  xyz P1S_D1u,  P1S_D1v,
      P2S_D1u,  P2S_D1v,
      P12S_D1u, P12S_D1v,
      PS_D1u,   PS_D1v;
  //
  this->Eval_D1_P1S  (u, v, P1S_D1u,  P1S_D1v);
  this->Eval_D1_P2S  (u, v, P2S_D1u,  P2S_D1v);
  this->Eval_D1_P12S (u, v, P12S_D1u, P12S_D1v);
  //
  PS_D1u = P1S_D1u + P2S_D1u - P12S_D1u;
  PS_D1v = P1S_D1v + P2S_D1v - P12S_D1v;

  if ( m_comp == EvalComponent_P1S )
  {
    dU = P1S_D1u;
    dV = P1S_D1v;
  }
  else if ( m_comp == EvalComponent_P2S )
  {
    dU = P2S_D1u;
    dV = P2S_D1v;
  }
  else if ( m_comp == EvalComponent_P12S )
  {
    dU = P12S_D1u;
    dV = P12S_D1v;
  }
  else // All components of the Boolean sum.
  {
    dU = PS_D1u;
    dV = PS_D1v;
  }
}

//-----------------------------------------------------------------------------

void mobius::geom_CoonsSurfaceCubic::Eval_P1S(const double u,
                                              const double v,
                                              xyz&         S) const
{
  bspl_HermiteLaw a0(0);
  bspl_HermiteLaw a1(1);
  bspl_HermiteLaw b0(2);
  bspl_HermiteLaw b1(3);

  S = a0.Eval(v) * m_Su0->Eval(u)
    + a1.Eval(v) * m_Su1->Eval(u)
    + b0.Eval(v) * m_dSu0_dv->Eval(u)
    + b1.Eval(v) * m_dSu1_dv->Eval(u);
}

//-----------------------------------------------------------------------------

void mobius::geom_CoonsSurfaceCubic::Eval_P2S(const double u,
                                              const double v,
                                              xyz&         S) const
{
  bspl_HermiteLaw a0(0);
  bspl_HermiteLaw a1(1);
  bspl_HermiteLaw b0(2);
  bspl_HermiteLaw b1(3);

  S = a0.Eval(u) * m_S0v->Eval(v)
    + a1.Eval(u) * m_S1v->Eval(v)
    + b0.Eval(u) * m_dS0v_du->Eval(v)
    + b1.Eval(u) * m_dS1v_du->Eval(v);
}

//-----------------------------------------------------------------------------

void mobius::geom_CoonsSurfaceCubic::Eval_P12S(const double u,
                                               const double v,
                                               xyz&         S) const
{
  // Compute Hermite polynomials.
  bspl_HermiteLaw a0(0);
  bspl_HermiteLaw a1(1);
  bspl_HermiteLaw b0(2);
  bspl_HermiteLaw b1(3);
  //
  const double a0u = a0.Eval(u);
  const double a1u = a1.Eval(u);
  const double b0u = b0.Eval(u);
  const double b1u = b1.Eval(u);

  const xyz row0 = this->evalS12Row(0, v);
  const xyz row1 = this->evalS12Row(1, v);
  const xyz row2 = this->evalS12Row(2, v);
  const xyz row3 = this->evalS12Row(3, v);
  //
  const xyz S1 = row0*a0u;
  const xyz S2 = row1*a1u;
  const xyz S3 = row2*b0u;
  const xyz S4 = row3*b1u;

  S = S1 + S2 + S3 + S4;
}

//-----------------------------------------------------------------------------

void mobius::geom_CoonsSurfaceCubic::Eval_D1_P1S(const double u,
                                                 const double v,
                                                 xyz&         dU,
                                                 xyz&         dV) const
{
  // Compute Hermite polynomials.
  bspl_HermiteLaw a0(0);
  bspl_HermiteLaw a1(1);
  bspl_HermiteLaw b0(2);
  bspl_HermiteLaw b1(3);
  //
  const double A0 = a0.Eval(v);
  const double A1 = a1.Eval(v);
  const double B0 = b0.Eval(v);
  const double B1 = b1.Eval(v);

  // Compute derivatives of constraints.
  const xyz dS1 = m_Su0->Eval_D1(u);
  const xyz dS2 = m_Su1->Eval_D1(u);
  const xyz dS3 = m_dSu0_dv->Eval_D1(u);
  const xyz dS4 = m_dSu1_dv->Eval_D1(u);

  xyz dP1S_dU = A0*dS1 + A1*dS2 + B0*dS3 + B1*dS4;
  xyz dP1S_dV = xyz(); // todo nyi

  dU = dP1S_dU;
  dV = dP1S_dV;
}

//-----------------------------------------------------------------------------

void mobius::geom_CoonsSurfaceCubic::Eval_D1_P2S(const double u,
                                                 const double v,
                                                 xyz&         dU,
                                                 xyz&         dV) const
{
  // Compute derivatives of Hermite polynomials.
  bspl_HermiteLaw a0(0);
  bspl_HermiteLaw a1(1);
  bspl_HermiteLaw b0(2);
  bspl_HermiteLaw b1(3);
  //
  const double dA0 = a0.Eval_D1(u);
  const double dA1 = a1.Eval_D1(u);
  const double dB0 = b0.Eval_D1(u);
  const double dB1 = b1.Eval_D1(u);

  // Compute constraints.
  const xyz S1 = m_S0v->Eval(v);
  const xyz S2 = m_S1v->Eval(v);
  const xyz S3 = m_dS0v_du->Eval(v);
  const xyz S4 = m_dS1v_du->Eval(v);

  xyz dP2S_dU = dA0*S1 + dA1*S2 + dB0*S3 + dB1*S4;
  xyz dP2S_dV = xyz(); // todo nyi

  dU = dP2S_dU;
  dV = dP2S_dV;
}

//-----------------------------------------------------------------------------

void mobius::geom_CoonsSurfaceCubic::Eval_D1_P12S(const double u,
                                                  const double v,
                                                  xyz&         dU,
                                                  xyz&         dV) const
{
  // Compute derivatives of Hermite polynomials.
  bspl_HermiteLaw a0(0);
  bspl_HermiteLaw a1(1);
  bspl_HermiteLaw b0(2);
  bspl_HermiteLaw b1(3);
  //
  const double dA0 = a0.Eval_D1(u);
  const double dA1 = a1.Eval_D1(u);
  const double dB0 = b0.Eval_D1(u);
  const double dB1 = b1.Eval_D1(u);

  const xyz row0 = this->evalS12Row(0, v);
  const xyz row1 = this->evalS12Row(1, v);
  const xyz row2 = this->evalS12Row(2, v);
  const xyz row3 = this->evalS12Row(3, v);

  xyz S1 = dA0*row0;
  xyz S2 = dA1*row1;
  xyz S3 = dB0*row2;
  xyz S4 = dB1*row3;

  dU = S1 + S2 + S3 + S4;
  dV = xyz(); // TODO nyi
}

//-----------------------------------------------------------------------------

mobius::xyz mobius::geom_CoonsSurfaceCubic::evalS12Row(const int    row,
                                                       const double v) const
{
  bspl_HermiteLaw a0(0);
  bspl_HermiteLaw a1(1);
  bspl_HermiteLaw b0(2);
  bspl_HermiteLaw b1(3);

  const double a0v = a0.Eval(v);
  const double a1v = a1.Eval(v);
  const double b0v = b0.Eval(v);
  const double b1v = b1.Eval(v);

  xyz res;
  switch ( row )
  {
    case 0:
      res = m_S00*a0v + m_S01*a1v + m_dS00_dv*b0v + m_dS01_dv*b1v;
      break;
    case 1:
      res = m_S10*a0v + m_S11*a1v + m_dS10_dv*b0v + m_dS11_dv*b1v;
      break;
    case 2:
      res = m_dS00_du*a0v + m_dS01_du*a1v + m_d2S_dudv00*b0v + m_d2S_dudv01*b1v;
      break;
    case 3:
      res = m_dS10_du*a0v + m_dS11_du*a1v + m_d2S_dudv10*b0v + m_d2S_dudv11*b1v;
      break;
    default: break;
  };
  return res;
}
