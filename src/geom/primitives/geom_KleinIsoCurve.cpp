//-----------------------------------------------------------------------------
// Created on: 16 December 2014
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
#include <mobius/geom_KleinIsoCurve.h>

// Geometry includes
#include <mobius/geom_KleinBottle.h>

// Standard includes
#include <math.h>

//! Constructor.
//! \param r     [in] radius of the hole.
//! \param type  [in] type of iso (U or V).
//! \param param [in] parameter value to freeze.
mobius::geom_KleinIsoCurve::geom_KleinIsoCurve(const double  r,
                                               const IsoType type,
                                               const double  param)
: geom_Curve(), m_fR(r), m_type(type), m_fParam(param)
{
}

//! Destructor.
mobius::geom_KleinIsoCurve::~geom_KleinIsoCurve()
{}

//! Calculates boundary box for the Klein isoparametric curve.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::geom_KleinIsoCurve::GetBounds(double& xMin, double& xMax,
                                           double& yMin, double& yMax,
                                           double& zMin, double& zMax) const
{
  xMin = -m_fR; xMax = m_fR;
  yMin = -m_fR; yMax = m_fR;
  zMin = -m_fR; zMax = m_fR;
}

//! \return min parameter.
double mobius::geom_KleinIsoCurve::GetMinParameter() const
{
  return 0.0;
}

//! \return max parameter.
double mobius::geom_KleinIsoCurve::GetMaxParameter() const
{
  return 2*M_PI;
}

//! Evaluates Klein iso-curve.
//! \param p [in] parameter value to evaluate curve for.
//! \param C [out] 3D point evaluated for the given parameter.
void mobius::geom_KleinIsoCurve::Eval(const double p,
                                      t_xyz&       C) const
{
  double u = ( (m_type == Iso_U) ? m_fParam : p );
  double v = ( (m_type == Iso_V) ? m_fParam : p );

  // Evaluate surface with fixed parameter
  double x, y, z;
  geom_KleinBottle::Eval(m_fR, u, v, x, y, z);

  // Set output parameter
  C.SetX(x);
  C.SetY(y);
  C.SetZ(z);
}
