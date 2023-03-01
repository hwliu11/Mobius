// Created on: 1992-09-28
// Created by: Didier PIFFAULT
// Copyright (c) 1992-1999 Matra Datavision
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

#ifndef _Intf_InterferencePolygon2d_HeaderFile
#define _Intf_InterferencePolygon2d_HeaderFile

#include <mobius/Intf_Interference.hxx>

namespace mobius {
namespace occ {

class Standard_OutOfRange;
class Intf_Polygon2d;
class gp_Pnt2d;

//! Computes the  interference between two  polygons or
//! the    self intersection of    a  polygon  in  two
//! dimensions.
class Intf_InterferencePolygon2d  : public Intf_Interference
{
public:

  //! Constructs an empty interference of Polygon.
  mobiusGeom_EXPORT Intf_InterferencePolygon2d();

  //! Constructs and computes an interference between two Polygons.
  mobiusGeom_EXPORT Intf_InterferencePolygon2d(const Intf_Polygon2d& Obje1, const Intf_Polygon2d& Obje2);

  //! Constructs and computes the auto interference of a Polygon.
  mobiusGeom_EXPORT Intf_InterferencePolygon2d(const Intf_Polygon2d& Obje);

  //! Computes an interference between two Polygons.
  mobiusGeom_EXPORT void Perform (const Intf_Polygon2d& Obje1, const Intf_Polygon2d& Obje2);

  //! Computes the self interference of a Polygon.
  mobiusGeom_EXPORT void Perform (const Intf_Polygon2d& Obje);

  //! Gives the  geometrical 2d point   of the  intersection
  //! point at address <Index> in the interference.
  mobiusGeom_EXPORT gp_Pnt2d Pnt2dValue (const int Index) const;

private:

  
  mobiusGeom_EXPORT void Interference (const Intf_Polygon2d& Obje1, const Intf_Polygon2d& Obje2);
  
  mobiusGeom_EXPORT void Interference (const Intf_Polygon2d& Obje);
  
  mobiusGeom_EXPORT void Clean();
  
  //! Computes the intersection between two segments
  //! <BegO><EndO> et <BegT><EndT>.
  mobiusGeom_EXPORT void Intersect (const int iO, const int iT, const gp_Pnt2d& BegO, const gp_Pnt2d& EndO, const gp_Pnt2d& BegT, const gp_Pnt2d& EndT);

  bool oClos;
  bool tClos;
  int nbso;

};

}
}

#endif // _Intf_InterferencePolygon2d_HeaderFile
