//-----------------------------------------------------------------------------
// Created on: 15 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
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
void mobius::geom_PositionCloud::Bounds(double& xMin, double& xMax,
                                        double& yMin, double& yMax,
                                        double& zMin, double& zMax) const
{
  double x_min = DBL_MAX, x_max = -DBL_MAX;
  double y_min = DBL_MAX, y_max = -DBL_MAX;
  double z_min = DBL_MAX, z_max = -DBL_MAX;

  // Iterate over the points to calculate bounds
  for ( int p = 0; p < (int) m_cloud.size(); ++p )
  {
    const xyz&   P = m_cloud.at(p);
    const double x = P.X(),
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
