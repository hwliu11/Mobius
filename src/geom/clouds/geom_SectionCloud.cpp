//-----------------------------------------------------------------------------
// Created on: 13 October 2013
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
#include <mobius/geom_SectionCloud.h>

//! Default constructor.
mobius::geom_SectionCloud::geom_SectionCloud() : geom_PointCloud()
{}

//! Constructs cloud wrapper under the given set of sections.
//! \param sections [in] points distributed by sections.
mobius::geom_SectionCloud::geom_SectionCloud(const std::vector< Ptr<geom_SectionLine> >& sections)
: geom_PointCloud()
{
  m_cloud = sections;
}

//! Constructs cloud wrapper under the given set of points distributed
//! by sections.
//! \param sections [in] points distributed by sections.
mobius::geom_SectionCloud::geom_SectionCloud(const std::vector< std::vector<xyz> >& sections)
: geom_PointCloud()
{
  for ( size_t s = 0; s < sections.size(); ++s )
    m_cloud.push_back( new geom_SectionLine( (int) s, sections[s] ) );
}

//! Destructor.
mobius::geom_SectionCloud::~geom_SectionCloud()
{}

//! Calculates boundary box for the point cloud.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::geom_SectionCloud::Bounds(double& xMin, double& xMax,
                                       double& yMin, double& yMax,
                                       double& zMin, double& zMax) const
{
  double x_min = DBL_MAX, x_max = -DBL_MAX;
  double y_min = DBL_MAX, y_max = -DBL_MAX;
  double z_min = DBL_MAX, z_max = -DBL_MAX;

  // Iterate over the points to calculate bounds
  for ( int s = 0; s < (int) m_cloud.size(); ++s )
  {
    // Access section
    const Ptr<geom_SectionLine>& S = m_cloud.at(s);

    // Iterate over the section points
    for ( size_t p = 0; p < S->Pts->NumberOfPoints(); ++p )
    {
      const xyz& P = S->Pts->Point(p);
      const double x = P.X(), y = P.Y(), z = P.Z();

      if ( x > x_max )
        x_max = x;
      if ( x < x_min )
        x_min = x;
      if ( y > y_max )
        y_max = y;
      if ( y < y_min )
        y_min = y;
      if ( z > z_max )
        z_max = z;
      if ( z < z_min )
        z_min = z;
    }
  }

  // Set results
  xMin = x_min;
  xMax = x_max;
  yMin = y_min;
  yMax = y_max;
  zMin = z_min;
  zMax = z_max;
}

//! Adds new section to the cloud.
//! \param section [in] point section to add.
void mobius::geom_SectionCloud::AddSection(const Ptr<geom_SectionLine>& section)
{
  m_cloud.push_back(section);
}

//! Returns number of sections.
//! \return number of sections.
size_t mobius::geom_SectionCloud::NumberOfSections() const
{
  return m_cloud.size();
}

//! Returns section with the given index in the internal collection.
//! \param idx [in] index of section to access.
//! \return requested section of cloud.
const mobius::Ptr<mobius::geom_SectionLine>&
  mobius::geom_SectionCloud::SectionByIndex(const size_t idx) const
{
  return m_cloud.at(idx);
}

//! Searches for a section with the given associated ID. The first occurrence
//! will be returned.
//! \param ID [in] ID of section to access.
//! \return requested section of cloud.
mobius::Ptr<mobius::geom_SectionLine>
  mobius::geom_SectionCloud::SectionByID(const int ID) const
{
  for ( size_t s = 0; s < m_cloud.size(); ++s )
  {
    if ( m_cloud[s]->ID == ID )
      return m_cloud[s];
  }
  return NULL;
}

//! Accessor for points.
//! \return double vector of points.
const std::vector< mobius::Ptr<mobius::geom_SectionLine> >&
  mobius::geom_SectionCloud::Sections() const
{
  return m_cloud;
}

//! Converts section cloud to nested collection of points.
//! \return nested collection of points.
std::vector< std::vector<mobius::xyz> > mobius::geom_SectionCloud::Points() const
{
  std::vector< std::vector<xyz> > pts;
  for ( size_t s = 0; s < m_cloud.size(); ++s )
  {
    const Ptr<sline>& sct_line = m_cloud[s];
    pts.push_back( sct_line->Pts->Points() );
  }
  return pts;
}

//! Converts section cloud to position cloud.
//! \return position cloud.
mobius::Ptr<mobius::pcloud> mobius::geom_SectionCloud::ToPositionCloud() const
{
  std::vector<xyz> pts;
  for ( size_t s = 0; s < m_cloud.size(); ++s )
  {
    const Ptr<sline>& sct_line = m_cloud[s];
    const std::vector<xyz>& section_pts = sct_line->Pts->Points();
    for ( size_t p = 0; p < section_pts.size(); ++p )
      pts.push_back(section_pts[p]);
  }
  return new pcloud(pts);
}
