//-----------------------------------------------------------------------------
// Created on: 15 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef geom_PointCloud_HeaderFile
#define geom_PointCloud_HeaderFile

// Geometry includes
#include <mobius/geom_Geometry.h>

namespace mobius {

//! Base class for point clouds.
class geom_PointCloud : public geom_Geometry
{
// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_PointCloud();

  mobiusGeom_EXPORT virtual
    ~geom_PointCloud();

};

};

#endif
