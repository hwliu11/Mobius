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

// Own include
#include <mobius/geom_BezierOnRailsSurface.h>

// STD includes
#include <math.h>

//! Constructor.
//! \param r [in] first rail curve.
//! \param c [in] middle weighted curve.
//! \param q [in] second rail curve.
//! \param w [in] weight law.
mobius::geom_BezierOnRailsSurface::geom_BezierOnRailsSurface(const Ptr<curve>&          r,
                                                             const Ptr<curve>&          c,
                                                             const Ptr<curve>&          q,
                                                             const Ptr<bspl_ScalarLaw>& w)
: geom_Surface(),
  m_r(r),
  m_c(c),
  m_q(q),
  m_w(w)
{
}

//! Destructor.
mobius::geom_BezierOnRailsSurface::~geom_BezierOnRailsSurface()
{}

//! Calculates boundary box for the surface.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::geom_BezierOnRailsSurface::Bounds(double& xMin, double& xMax,
                                               double& yMin, double& yMax,
                                               double& zMin, double& zMax) const
{
  /* =======================================
   *  This approach is very approximate (!)
   * ======================================= */

  // r(u)
  double r_x_min, r_x_max, r_y_min, r_y_max, r_z_min, r_z_max;
  m_r->Bounds(r_x_min, r_x_max, r_y_min, r_y_max, r_z_min, r_z_max);

  // c(u)
  double c_x_min, c_x_max, c_y_min, c_y_max, c_z_min, c_z_max;
  m_c->Bounds(c_x_min, c_x_max, c_y_min, c_y_max, c_z_min, c_z_max);

  // q(u)
  double q_x_min, q_x_max, q_y_min, q_y_max, q_z_min, q_z_max;
  m_q->Bounds(q_x_min, q_x_max, q_y_min, q_y_max, q_z_min, q_z_max);

  // Set results
  xMin = fmin(r_x_min, fmin(c_x_min, q_x_min));
  xMax = fmax(r_x_max, fmax(c_x_max, q_x_max));
  yMin = fmin(r_y_min, fmin(c_y_min, q_y_min));
  yMax = fmax(r_y_max, fmax(c_y_max, q_y_max));
  zMin = fmin(r_z_min, fmin(c_z_min, q_z_min));
  zMax = fmax(r_z_max, fmax(c_z_max, q_z_max));
}

//! Returns minimal U parameter.
//! \return parameter value.
double mobius::geom_BezierOnRailsSurface::MinParameter_U() const
{
  return m_c->MinParameter();
}

//! Returns maximal U parameter.
//! \return parameter value.
double mobius::geom_BezierOnRailsSurface::MaxParameter_U() const
{
  return m_c->MaxParameter();
}

//! Returns minimal V parameter.
//! \return parameter value.
double mobius::geom_BezierOnRailsSurface::MinParameter_V() const
{
  return 0.0;
}

//! Returns maximal V parameter.
//! \return parameter value.
double mobius::geom_BezierOnRailsSurface::MaxParameter_V() const
{
  return 1.0;
}

//! Evaluates surface in the given parametric point (u, v).
//! \param u [in]  first parameter.
//! \param v [in]  second parameter.
//! \param S [out] evaluated spatial point S(u, v).
void mobius::geom_BezierOnRailsSurface::Eval(const double u,
                                             const double v,
                                             xyz&         S) const
{
  xyz S1;
  double S2;

  this->eval_S1(u, v, S1);
  this->eval_S2(u, v, S2);

  S = S1/S2;
}

//-----------------------------------------------------------------------------
// Internal methods
//-----------------------------------------------------------------------------

//! Evaluates numerator.
//! \param u [in]  first parameter.
//! \param v [in]  second parameter.
//! \param P [out] evaluated spatial point S(u, v).
void mobius::geom_BezierOnRailsSurface::eval_S1(const double u,
                                                const double v,
                                                xyz&         P) const
{
  xyz P_r, P_c, P_q;
  double w = m_w->Eval(u);

  m_r->Eval(u, P_r);
  m_c->Eval(u, P_c);
  m_q->Eval(u, P_q);

  P = P_r*(1-v)*(1-v) + P_c*2*v*(1-v)*w + P_q*v*v;
}

//! Evaluates denominator.
//! \param u   [in]  first parameter.
//! \param v   [in]  second parameter.
//! \param val [out] evaluated value.
void mobius::geom_BezierOnRailsSurface::eval_S2(const double u,
                                                const double v,
                                                double&      val) const
{
  double w = m_w->Eval(u);

  val = (1-v)*(1-v) + 2*v*(1-v)*w + v*v;
}
