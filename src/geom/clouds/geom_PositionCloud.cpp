//-----------------------------------------------------------------------------
// Created on: 15 November 2013
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
#include <mobius/geom_PositionCloud.h>

//! Default constructor.
mobius::geom_PositionCloud::geom_PositionCloud() : geom_PointCloud()
{}

//! Constructor accepting points of the cloud.
//! \param pts [in] point to populate the cloud with.
mobius::geom_PositionCloud::geom_PositionCloud(const std::vector<xyz>& pts) : geom_PointCloud()
{
  this->SetPoints(pts);
}

//! Destructor.
mobius::geom_PositionCloud::~geom_PositionCloud()
{}

//! Calculates boundary box for the point cloud.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::geom_PositionCloud::Bounds(adouble& xMin, adouble& xMax,
                                        adouble& yMin, adouble& yMax,
                                        adouble& zMin, adouble& zMax) const
{
  adouble x_min = DBL_MAX, x_max = -DBL_MAX;
  adouble y_min = DBL_MAX, y_max = -DBL_MAX;
  adouble z_min = DBL_MAX, z_max = -DBL_MAX;

  // Iterate over the points to calculate bounds
  for ( int p = 0; p < (int) m_cloud.size(); ++p )
  {
    const xyz&   P = m_cloud.at(p);
    const adouble x = P.X(),
                 y = P.Y(),
                 z = P.Z();

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

  // Set results
  xMin = x_min;
  xMax = x_max;
  yMin = y_min;
  yMax = y_max;
  zMin = z_min;
  zMax = z_max;
}

//! Adds new point to the cloud.
//! \param section [in] point to add.
void mobius::geom_PositionCloud::AddPoint(const xyz& point)
{
  m_cloud.push_back(point);
}

//! Returns number of points.
//! \return number of points.
size_t mobius::geom_PositionCloud::NumberOfPoints() const
{
  return m_cloud.size();
}

//! Returns point with the given index.
//! \param idx [in] index of point to access.
//! \return requested point.
const mobius::xyz& mobius::geom_PositionCloud::Point(const size_t idx) const
{
  return m_cloud.at(idx);
}

//! Sets point cloud.
//! \param cloud [in] points to set.
void mobius::geom_PositionCloud::SetPoints(const std::vector<xyz>& cloud)
{
  m_cloud = cloud;
}

//! Returns internal collection of points.
//! \return internal collection of points.
const std::vector<mobius::xyz>& mobius::geom_PositionCloud::Points() const
{
  return m_cloud;
}

//! Cleans up the cloud.
void mobius::geom_PositionCloud::Clear()
{
  m_cloud.clear();
}
