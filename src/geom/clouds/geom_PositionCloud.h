//-----------------------------------------------------------------------------
// Created on: 15 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef geom_PositionCloud_HeaderFile
#define geom_PositionCloud_HeaderFile

// Geometry includes
#include <mobius/geom_PointCloud.h>

// Core includes
#include <mobius/core_XYZ.h>

// STL includes
#include <vector>

namespace mobius {

//! Represents simple point cloud as a collection of points.
class geom_PositionCloud : public geom_PointCloud
{
// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_PositionCloud();

  mobiusGeom_EXPORT
    geom_PositionCloud(const std::vector<core_XYZ>& pts);

  mobiusGeom_EXPORT virtual
    ~geom_PositionCloud();

public:

  mobiusGeom_EXPORT virtual void
    Bounds(double& xMin, double& xMax,
           double& yMin, double& yMax,
           double& zMin, double& zMax) const;

public:

  mobiusGeom_EXPORT virtual void
    AddPoint(const core_XYZ& point);

  mobiusGeom_EXPORT virtual size_t
    NumberOfPoints() const;

  mobiusGeom_EXPORT virtual const core_XYZ&
    Point(const size_t idx) const;

  mobiusGeom_EXPORT virtual void
    SetPoints(const std::vector<core_XYZ>& cloud);

  mobiusGeom_EXPORT virtual const std::vector<core_XYZ>&
    Points() const;

  mobiusGeom_EXPORT virtual void
    Clear();

protected:

  //! Actual collection of points.
  std::vector<core_XYZ> m_cloud;

};

//! Handy alias for double type.
typedef geom_PositionCloud pcloud;

};

#endif
