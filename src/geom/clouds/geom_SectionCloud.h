//-----------------------------------------------------------------------------
// Created on: 13 October 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef geom_SectionCloud_HeaderFile
#define geom_SectionCloud_HeaderFile

// Geometry includes
#include <mobius/geom_SectionLine.h>

// Core includes
#include <mobius/core_XYZ.h>

// STL includes
#include <vector>

namespace mobius {

//! Represents point cloud arranged as ordered sections of points.
class geom_SectionCloud : public geom_PointCloud
{
// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_SectionCloud();

  mobiusGeom_EXPORT
    geom_SectionCloud(const std::vector< Ptr<geom_SectionLine> >& sections);

  mobiusGeom_EXPORT
    geom_SectionCloud(const std::vector< std::vector<xyz> >& sections);

  mobiusGeom_EXPORT virtual
    ~geom_SectionCloud();

public:

  mobiusGeom_EXPORT virtual void
    Bounds(double& xMin, double& xMax,
           double& yMin, double& yMax,
           double& zMin, double& zMax) const;

public:

  mobiusGeom_EXPORT void
    AddSection(const Ptr<geom_SectionLine>& section);

  mobiusGeom_EXPORT size_t
    NumberOfSections() const;

  mobiusGeom_EXPORT const Ptr<geom_SectionLine>&
    SectionByIndex(const size_t idx) const;

  mobiusGeom_EXPORT Ptr<geom_SectionLine>
    SectionByID(const int ID) const;

  mobiusGeom_EXPORT const std::vector< Ptr<geom_SectionLine> >&
    Sections() const;

  mobiusGeom_EXPORT std::vector< std::vector<xyz> >
    Points() const;

  mobiusGeom_EXPORT Ptr<pcloud>
    ToPositionCloud() const;

private:

  //! Actual collection of points distributed by sections.
  std::vector< Ptr<geom_SectionLine> > m_cloud;

};

//! Handy shortcut.
typedef geom_SectionCloud scloud;

};

#endif
