//-----------------------------------------------------------------------------
// Created on: 23 May 2013
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
#include <mobius/geom_Link.h>

//! Complete constructor.
//! \param P1 [in] first point.
//! \param P2 [in] second point.
mobius::geom_Link::geom_Link(const xyz& P1,
                             const xyz& P2) : geom_Geometry()
{
  m_p1 = P1;
  m_p2 = P2;
}

//! Destructor.
mobius::geom_Link::~geom_Link()
{}

//! Calculates boundary box for the link.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::geom_Link::Bounds(adouble& xMin, adouble& xMax,
                               adouble& yMin, adouble& yMax,
                               adouble& zMin, adouble& zMax) const
{
  adouble x_min = DBL_MAX, x_max = -DBL_MAX;
  adouble y_min = DBL_MAX, y_max = -DBL_MAX;
  adouble z_min = DBL_MAX, z_max = -DBL_MAX;

  const adouble x[] = { m_p1.X(), m_p2.X() };
  const adouble y[] = { m_p1.Y(), m_p2.Y() };
  const adouble z[] = { m_p1.Z(), m_p2.Z() };

  for ( int c = 0; c < 2; ++c )
  {
    if ( x[c] > x_max )
      x_max = x[c];
    if ( x[c] < x_min )
      x_min = x[c];
    if ( y[c] > y_max )
      y_max = y[c];
    if ( y[c] < y_min )
      y_min = y[c];
    if ( z[c] > z_max )
      z_max = z[c];
    if ( z[c] < z_min )
      z_min = z[c];
  }

  // Set results
  xMin = x_min;
  xMax = x_max;
  yMin = y_min;
  yMax = y_max;
  zMin = z_min;
  zMax = z_max;
}
