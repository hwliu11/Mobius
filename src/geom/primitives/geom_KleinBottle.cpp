//-----------------------------------------------------------------------------
// Created on: 15 December 2014
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
#include <mobius/geom_KleinBottle.h>

// Standard includes
#include <math.h>

//! Constructor.
//! \param r [in] radius of the hole.
mobius::geom_KleinBottle::geom_KleinBottle(const double r)
: geom_Surface(), m_fR(r)
{}

//! Destructor.
mobius::geom_KleinBottle::~geom_KleinBottle()
{}

//! Calculates boundary box for the Klein bottle.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::geom_KleinBottle::Bounds(double& xMin, double& xMax,
                                      double& yMin, double& yMax,
                                      double& zMin, double& zMax) const
{
  xMin = -10.0; xMax = 10.0;
  yMin = -10.0; yMax = 10.0;
  zMin = -10.0; zMax = 10.0;
}

//! \return min U parameter.
double mobius::geom_KleinBottle::MinParameter_U() const
{
  return 0.0;
}

//! \return max U parameter.
double mobius::geom_KleinBottle::MaxParameter_U() const
{
  return 2*M_PI;
}

//! \return min V parameter.
double mobius::geom_KleinBottle::MinParameter_V() const
{
  return 0.0;
}

//! \return max V parameter.
double mobius::geom_KleinBottle::MaxParameter_V() const
{
  return 2*M_PI;
}

//! Evaluates Klein bottle.
//! \param u [in] U parameter value to evaluate surface for.
//! \param v [in] V parameter value to evaluate surface for.
//! \param C [out] 3D point corresponding to the given parameter pair.
void mobius::geom_KleinBottle::Eval(const double u,
                                    const double v,
                                    xyz&         C) const
{
  double x, y, z;
  Eval(m_fR, u, v, x, y, z);

  // Set output parameter
  C.SetX(x);
  C.SetY(y);
  C.SetZ(z);
}

//! Extracts isoparametric curve corresponding to the passed {u} level.
//! \param u [in] parameter value to extract isoparametric curve for.
//! \return iso-line.
mobius::ptr<mobius::geom_KleinIsoCurve>
  mobius::geom_KleinBottle::Iso_U(const double u) const
{
  ptr<geom_KleinIsoCurve>
    Iso = new geom_KleinIsoCurve(m_fR, geom_KleinIsoCurve::Iso_U, u);
  return Iso;
}

//! Extracts isoparametric curve corresponding to the passed {v} level.
//! \param v [in] parameter value to extract isoparametric curve for.
//! \return iso-line.
mobius::ptr<mobius::geom_KleinIsoCurve>
  mobius::geom_KleinBottle::Iso_V(const double v) const
{
  ptr<geom_KleinIsoCurve>
    Iso = new geom_KleinIsoCurve(m_fR, geom_KleinIsoCurve::Iso_V, v);
  return Iso;
}

//! Evaluates Klein bottle surface against the given pair of parameters.
//! \param r [in]  hole radius.
//! \param u [in]  U parameter.
//! \param v [in]  V parameter.
//! \param x [out] output X coordinate.
//! \param y [out] output Y coordinate.
//! \param z [out] output Z coordinate.
void mobius::geom_KleinBottle::Eval(const double r,
                                    const double u,
                                    const double v,
                                    double&      x,
                                    double&      y,
                                    double&      z)
{
  x = (r + cos(u/2)*sin(v) - sin(u/2)*sin(2*v))*cos(u);
  y = (r + cos(u/2)*sin(v) - sin(u/2)*sin(2*v))*sin(u);
  z = sin(u/2)*sin(v) + cos(u/2)*sin(2*v);
}
