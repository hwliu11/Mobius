// Created on: 2012-02-10
// Created by: Serey ZERCHANINOV
// Copyright (c) 2012-2014 OPEN CASCADE SAS
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

#ifndef _Intf_Polygon2d_HeaderFile
#define _Intf_Polygon2d_HeaderFile

#include <mobius/Bnd_Box2d.hxx>

class Bnd_Box2d;
class gp_Pnt2d;

//! Describes the necessary polygon information to compute
//! the interferences.
class Intf_Polygon2d 
{
public:

  //! Returns the bounding box of the polygon.
    const Bnd_Box2d& Bounding() const;
  
  //! Returns True if the polyline is closed.
  mobiusGeom_EXPORT virtual bool Closed() const;
  virtual ~Intf_Polygon2d() {}
  
  //! Returns the tolerance of the polygon.
  mobiusGeom_EXPORT virtual double DeflectionOverEstimation() const = 0;
  
  //! Returns the number of Segments in the polyline.
  mobiusGeom_EXPORT virtual int NbSegments() const = 0;
  
  //! Returns the points of the segment <Index> in the Polygon.
  mobiusGeom_EXPORT virtual void Segment (const int theIndex, gp_Pnt2d& theBegin, gp_Pnt2d& theEnd) const = 0;


  Bnd_Box2d myBox;


};

inline const Bnd_Box2d& Intf_Polygon2d::Bounding () const
{
  return myBox;
}

#endif // _Intf_Polygon2d_HeaderFile
