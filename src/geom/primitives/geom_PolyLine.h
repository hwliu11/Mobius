//-----------------------------------------------------------------------------
// Created on: 08 October 2015
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef geom_PolyLine_HeaderFile
#define geom_PolyLine_HeaderFile

// Geometry includes
#include <mobius/geom_Link.h>

namespace mobius {

//! Non-parametric polyline.
class geom_PolyLine : public geom_Geometry
{
// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_PolyLine();

  mobiusGeom_EXPORT virtual
    ~geom_PolyLine();

public:

  mobiusGeom_EXPORT virtual void
    Bounds(double& xMin, double& xMax,
           double& yMin, double& yMax,
           double& zMin, double& zMax) const;

public:

  mobiusGeom_EXPORT void
    AddLink(const Ptr<geom_Link>& link);

  mobiusGeom_EXPORT int
    NumLinks() const;

  mobiusGeom_EXPORT const std::vector< Ptr<geom_Link> >&
    Links() const;

  mobiusGeom_EXPORT const Ptr<geom_Link>&
    Link(const size_t idx) const;

private:

  std::vector< Ptr<geom_Link> > m_links; //!< Links.

};

};

#endif
