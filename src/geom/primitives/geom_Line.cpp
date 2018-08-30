//-----------------------------------------------------------------------------
// Created on: 22 May 2014
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
#include <mobius/geom_Line.h>

//! Constructor.
//! \param origin [in] origin of line.
//! \param dir    [in] direction vector for line.
mobius::geom_Line::geom_Line(const xyz& origin,
                             const xyz& dir)
: geom_Curve(),
  m_origin(origin),
  m_dir( dir.Normalized() )
{
}

//! Destructor.
mobius::geom_Line::~geom_Line()
{}

//! Calculates boundary box for the line.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::geom_Line::Bounds(adouble& xMin, adouble& xMax,
                               adouble& yMin, adouble& yMax,
                               adouble& zMin, adouble& zMax) const
{
  // Even though it is possible to reduce infinite space to something more
  // representative for line, we do not do it as we do not have any
  // practical need for that
  xMin = -DBL_MAX;
  yMin = -DBL_MAX;
  zMin = -DBL_MAX;
  xMax =  DBL_MAX;
  yMax =  DBL_MAX;
  zMax =  DBL_MAX;
}

//! Returns minimal parameter value.
//! \return minimal parameter value.
adouble mobius::geom_Line::MinParameter() const
{
  return -DBL_MAX;
}

//! Returns maximal parameter value.
//! \return maximal parameter value.
adouble mobius::geom_Line::MaxParameter() const
{
  return DBL_MAX;
}

//! Evaluates line for the given parameter.
//! \param u [in]  parameter value to evaluate curve for.
//! \param P [out] 3D point corresponding to the given parameter on curve.
void mobius::geom_Line::Eval(const adouble u,
                             xyz&         P) const
{
  P = m_origin + m_dir*u;
}
