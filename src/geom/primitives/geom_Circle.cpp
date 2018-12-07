//-----------------------------------------------------------------------------
// Created on: 05 September 2014
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
#include <mobius/geom_Circle.h>

// Standard includes
#include <math.h>

//! Constructor.
//! \param radius [in] radius of the circle.
//! \param tChain [in] transformation chain to apply.
mobius::geom_Circle::geom_Circle(const double                  radius,
                                 const core_IsoTransformChain& tChain)
: geom_Curve(tChain),
  m_fRadius(radius)
{}

//! Destructor.
mobius::geom_Circle::~geom_Circle()
{}

//! Calculates boundary box for the circle.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::geom_Circle::GetBounds(double& xMin, double& xMax,
                                    double& yMin, double& yMax,
                                    double& zMin, double& zMax) const
{
  // TODO: NYI
  xMin = -DBL_MAX;
  yMin = -DBL_MAX;
  zMin = -DBL_MAX;
  xMax =  DBL_MAX;
  yMax =  DBL_MAX;
  zMax =  DBL_MAX;
}

//! Returns minimal parameter value.
//! \return minimal parameter value.
double mobius::geom_Circle::GetMinParameter() const
{
  return 0.0;
}

//! Returns maximal parameter value.
//! \return maximal parameter value.
double mobius::geom_Circle::GetMaxParameter() const
{
  return 2*M_PI;
}

//! Evaluates circle for the given parameter.
//! \param u [in] parameter value to evaluate curve for.
//! \param P [out] 3D point corresponding to the given parameter on curve.
void mobius::geom_Circle::Eval(const double u,
                               xyz&         P) const
{
  const double x = m_fRadius*cos(u);
  const double y = m_fRadius*sin(u);
  const double z = 0.0;

  xyz P_local(x, y, z);
  P = m_tChain.Apply(P_local);
}
