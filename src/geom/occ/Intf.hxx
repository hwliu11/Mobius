// Created on: 1991-05-23
// Created by: Didier PIFFAULT
// Copyright (c) 1991-1999 Matra Datavision
// Copyright (c) 1999-2014 OPEN CASCADE SAS
//
// This file is part of Open CASCADE Technology software library.
//
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with special exception defined in the file
// OCCT_LGPL_EXCEPTION.txt. Consult the file LICENSE_LGPL_21.txt included in OCCT
// distribution for complete text of the license and disclaimer of any warranty.
//
// Alternatively, this file may be used under the terms of Open CASCADE
// commercial license or contractual agreement.

#ifndef _Intf_HeaderFile
#define _Intf_HeaderFile

#include <mobius/geom.h>

namespace mobius {
namespace occ {

class gp_Pnt;
class gp_XYZ;
class Intf_Polygon2d;
class Intf_SectionPoint;
class Intf_SectionLine;
class Intf_TangentZone;
class Intf_Interference;
class Intf_InterferencePolygon2d;

//! Interference computation  between polygons, lines  and
//! polyhedra with only  triangular  facets. These objects
//! are polygonal  representations of complex   curves and
//! triangulated representations of complex surfaces.
class Intf 
{
public:

  //! Gives the plane equation of the triangle <P1> <P2> <P3>.
  mobiusGeom_EXPORT static void
    PlaneEquation(const gp_Pnt& P1,
                  const gp_Pnt& P2,
                  const gp_Pnt& P3,
                  gp_XYZ& NormalVector,
                  double& PolarDistance);

  //! Computes if the triangle <P1> <P2> <P3> contain <ThePnt>.
  mobiusGeom_EXPORT static bool
    Contain(const gp_Pnt& P1, const gp_Pnt& P2, const gp_Pnt& P3, const gp_Pnt& ThePnt);

private:

friend class Intf_Polygon2d;
friend class Intf_SectionPoint;
friend class Intf_SectionLine;
friend class Intf_TangentZone;
friend class Intf_Interference;
friend class Intf_InterferencePolygon2d;

};

}
}

#endif // _Intf_HeaderFile
