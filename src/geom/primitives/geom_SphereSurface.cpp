//-----------------------------------------------------------------------------
// Created on: 05 September 2014
//-----------------------------------------------------------------------------
// Copyright (c) 2014-present, Sergey Slyadnev
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
#include <mobius/geom_SphereSurface.h>

// STD includes
#include <math.h>

//! Constructs surface by origin point and radius.
//! \param radius [in] radius of the sphere.
//! \param tChain [in] transformation chain to apply.
mobius::geom_SphereSurface::geom_SphereSurface(const double                  radius,
                                               const core_IsoTransformChain& tChain)
: geom_Surface(tChain),
  m_fRadius(radius)
{
}

//! Destructor.
mobius::geom_SphereSurface::~geom_SphereSurface()
{}

//! Calculates boundary box for the surface.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::geom_SphereSurface::GetBounds(double& xMin, double& xMax,
                                           double& yMin, double& yMax,
                                           double& zMin, double& zMax) const
{
  t_xyz global_center = m_tChain.Apply( this->Center() );
  t_xyz rad_pt(m_fRadius, m_fRadius, m_fRadius);

  t_xyz min_pt = global_center - rad_pt;
  t_xyz max_pt = global_center + rad_pt;

  xMin = min_pt.X();
  xMax = max_pt.X();
  yMin = min_pt.Y();
  yMax = max_pt.Y();
  zMin = min_pt.Z();
  zMax = max_pt.Z();
}

//! Returns minimal U parameter.
//! \return parameter value.
double mobius::geom_SphereSurface::GetMinParameter_U() const
{
  return 0.0;
}

//! Returns maximal U parameter.
//! \return parameter value.
double mobius::geom_SphereSurface::GetMaxParameter_U() const
{
  return 2*M_PI;
}

//! Returns minimal V parameter.
//! \return parameter value.
double mobius::geom_SphereSurface::GetMinParameter_V() const
{
  return -M_PI;
}

//! Returns maximal V parameter.
//! \return parameter value.
double mobius::geom_SphereSurface::GetMaxParameter_V() const
{
  return M_PI;
}

//! Evaluates surface in the given parametric point (u, v).
//! \param u [in]  first parameter.
//! \param v [in]  second parameter.
//! \param P [out] evaluated spatial point S(u, v).
void mobius::geom_SphereSurface::Eval(const double u,
                                      const double v,
                                      t_xyz&       S) const
{
  /* ==========================
   *  Sphere point in local CS
   * ========================== */

  const double x = m_fRadius*cos(v)*cos(u);
  const double y = m_fRadius*cos(v)*sin(u);
  const double z = m_fRadius*sin(v);

  S.SetX(x);
  S.SetY(y);
  S.SetZ(z);

  /* =====================
   *  Switch to global CS
   * ===================== */

  S = m_tChain.Apply(S);
}

//! Returns isoparametric curve for a fixed {u}.
//! \param u [in] fixed parameter.
//! \return isoparametric curve.
mobius::t_ptr<mobius::geom_Circle>
  mobius::geom_SphereSurface::Iso_U(const double u) const
{
  const double r             = m_fRadius; // Radius is the same
  const double ang_around_OX = M_PI/2.0;  // Flip circle vertical
  const double ang_around_OZ = u;         // Rotate to have iso-u

  // Rotation to represent iso-u in a local system of axes of sphere
  core_Quaternion qn_around_OX(t_xyz::OX(), ang_around_OX);
  core_Quaternion qn_around_OZ(t_xyz::OZ(), ang_around_OZ);
  core_Quaternion qn_local = qn_around_OZ * qn_around_OX;

  // Prepare transformation chain for the resulting isoline
  core_IsoTransformChain tChain;
  tChain << m_tChain << core_IsoTransform( qn_local, t_xyz() );

  // Return result
  return new geom_Circle(r, tChain);
}

//! Returns isoparametric curve for a fixed {v}.
//! \param v [in] fixed parameter.
//! \return isoparametric curve.
mobius::t_ptr<mobius::geom_Circle>
  mobius::geom_SphereSurface::Iso_V(const double v) const
{
  const double r  = m_fRadius*cos(v); // Radius is the same
  const double dZ = m_fRadius*sin(v); // Elevation

  // Transformation to apply in local CS of sphere
  t_xyz elevation(0.0, 0.0, dZ);
  core_IsoTransform T_local(core_Quaternion(), elevation);

  // Prepare transformation chain for the resulting isoline
  core_IsoTransformChain tChain;
  tChain << m_tChain << T_local;

  // Return result
  return new geom_Circle(r, tChain);
}
