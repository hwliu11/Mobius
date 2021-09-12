// Created on: 1991-06-18
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

#ifndef _Intf_TangentZone_HeaderFile
#define _Intf_TangentZone_HeaderFile

#include <Standard.hxx>
#include <Standard_DefineAlloc.hxx>
#include <Standard_Handle.hxx>

#include <mobius/Intf_SeqOfSectionPoint.hxx>
#include <double.hxx>
#include <int.hxx>
#include <bool.hxx>
class Standard_OutOfRange;
class Intf_SectionPoint;


//! Describes   a  zone  of  tangence  between  polygons  or
//! polyhedra as a sequence of points of intersection.
class Intf_TangentZone 
{
public:

  DEFINE_STANDARD_ALLOC

  
  //! Returns number of SectionPoint in this TangentZone.
    int NumberOfPoints() const;
  
  //! Gives   the   SectionPoint   of  address  <Index>  in  the
  //! TangentZone.
  mobiusGeom_EXPORT const Intf_SectionPoint& GetPoint (const int Index) const;
  
  //! Compares two TangentZones.
  mobiusGeom_EXPORT bool IsEqual (const Intf_TangentZone& Other) const;
bool operator == (const Intf_TangentZone& Other) const
{
  return IsEqual(Other);
}
  
  //! Checks if <ThePI> is in TangentZone.
  mobiusGeom_EXPORT bool Contains (const Intf_SectionPoint& ThePI) const;
  
  //! Gives  the parameter range of the  TangentZone on the first
  //! argument of the Interference. (Usable only for polygon)
    void ParamOnFirst (double& paraMin, double& paraMax) const;
  
  //! Gives the parameter range of the  TangentZone on the second
  //! argument of the Interference. (Usable only for polygon)
    void ParamOnSecond (double& paraMin, double& paraMax) const;
  
  //! Gives information  about  the    first argument   of   the
  //! Interference. (Usable only for polygon)
  mobiusGeom_EXPORT void InfoFirst (int& segMin, double& paraMin, int& segMax, double& paraMax) const;
  
  //! Gives   informations  about  the  second   argument of  the
  //! Interference. (Usable only for polygon)
  mobiusGeom_EXPORT void InfoSecond (int& segMin, double& paraMin, int& segMax, double& paraMax) const;
  
  //! Returns True if  <ThePI>  is in the parameter  range of the
  //! TangentZone.
  mobiusGeom_EXPORT bool RangeContains (const Intf_SectionPoint& ThePI) const;
  
  //! Returns True if the TangentZone  <Other> has  a common part
  //! with <me>.
  mobiusGeom_EXPORT bool HasCommonRange (const Intf_TangentZone& Other) const;
  
  //! Builds an empty tangent zone.
  mobiusGeom_EXPORT Intf_TangentZone();

  //! Adds a SectionPoint to the TangentZone.
  mobiusGeom_EXPORT void Append (const Intf_SectionPoint& Pi);
  
  //! Adds the TangentZone <Tzi> to <me>.
  mobiusGeom_EXPORT void Append (const Intf_TangentZone& Tzi);
  
  //! Inserts a SectionPoint in the TangentZone.
  mobiusGeom_EXPORT bool Insert (const Intf_SectionPoint& Pi);
  
  //! Inserts a point in the polygonal TangentZone.
  mobiusGeom_EXPORT void PolygonInsert (const Intf_SectionPoint& Pi);
  
  //! Inserts a SectionPoint before <Index> in the TangentZone.
  mobiusGeom_EXPORT void InsertBefore (const int Index, const Intf_SectionPoint& Pi);
  
  //! Inserts a SectionPoint after <Index> in the TangentZone.
  mobiusGeom_EXPORT void InsertAfter (const int Index, const Intf_SectionPoint& Pi);
  
  mobiusGeom_EXPORT void Dump (const int Indent) const;




protected:





private:



  Intf_SeqOfSectionPoint Result;
  double ParamOnFirstMin;
  double ParamOnFirstMax;
  double ParamOnSecondMin;
  double ParamOnSecondMax;


};


#include <mobius/Intf_TangentZone.lxx>





#endif // _Intf_TangentZone_HeaderFile
