//-----------------------------------------------------------------------------
// Created on: 08 October 2015
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
#include <mobius/geom_PolyLine.h>

//! Constructor.
mobius::geom_PolyLine::geom_PolyLine() : geom_Geometry()
{}

//! Destructor.
mobius::geom_PolyLine::~geom_PolyLine()
{}

//! Calculates boundary box for the line.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::geom_PolyLine::GetBounds(double& xMin, double& xMax,
                                      double& yMin, double& yMax,
                                      double& zMin, double& zMax) const
{
  // Even though it is possible to reduce infinite space to something more
  // representative for polyline, we do not do it as we do not have any
  // practical need for that
  xMin = -DBL_MAX;
  yMin = -DBL_MAX;
  zMin = -DBL_MAX;
  xMax =  DBL_MAX;
  yMax =  DBL_MAX;
  zMax =  DBL_MAX;
}

//! Adds the passed link to collection.
//! \param link [in] link to add.
void mobius::geom_PolyLine::AddLink(const t_ptr<geom_Link>& link)
{
  m_links.push_back(link);
}

//! \return number of links.
int mobius::geom_PolyLine::NumLinks() const
{
  return (int) ( m_links.size() );
}

//! \return all links.
const std::vector< mobius::t_ptr<mobius::geom_Link> >&
  mobius::geom_PolyLine::GetLinks() const
{
  return m_links;
}

//! Returns a link with the given 0-based index.
//! \param idx [in] index of the link to access.
//! \return link.
const mobius::t_ptr<mobius::geom_Link>&
  mobius::geom_PolyLine::GetLink(const size_t idx) const
{
  return m_links[idx];
}
