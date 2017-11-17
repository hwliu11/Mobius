//-----------------------------------------------------------------------------
// Created on: 08 October 2015
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
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
void mobius::geom_PolyLine::Bounds(double& xMin, double& xMax,
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
void mobius::geom_PolyLine::AddLink(const Ptr<geom_Link>& link)
{
  m_links.push_back(link);
}

//! \return number of links.
int mobius::geom_PolyLine::NumLinks() const
{
  return (int) ( m_links.size() );
}

//! \return all links.
const std::vector< mobius::Ptr<mobius::geom_Link> >&
  mobius::geom_PolyLine::Links() const
{
  return m_links;
}

//! Returns a link with the given 0-based index.
//! \param idx [in] index of the link to access.
//! \return link.
const mobius::Ptr<mobius::geom_Link>&
  mobius::geom_PolyLine::Link(const size_t idx) const
{
  return m_links[idx];
}
