// Created on: 1991-06-24
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

#ifndef _Intf_Interference_HeaderFile
#define _Intf_Interference_HeaderFile

#include <mobius/geom.h>

// CAS.CADE includes
#include <mobius/Intf_SeqOfSectionPoint.hxx>
#include <mobius/Intf_SeqOfSectionLine.hxx>
#include <mobius/Intf_SeqOfTangentZone.hxx>

class Standard_OutOfRange;
class Intf_SectionPoint;
class Intf_SectionLine;
class Intf_TangentZone;

//! Describes the   Interference  computation    result
//! between polygon2d or polygon3d or polyhedron
//! (as  three sequences   of  points  of  intersection,
//! polylines of intersection and zones de tangence).
class Intf_Interference 
{
public:

  //! Gives the number   of  points of  intersection  in the
  //! interference.
    int NbSectionPoints() const;
  
  //! Gives the point of  intersection of address  Index in
  //! the interference.
    const Intf_SectionPoint& PntValue (const int Index) const;
  
  //! Gives the number  of polylines of  intersection in the
  //! interference.
    int NbSectionLines() const;
  
  //! Gives the polyline of intersection at address <Index> in
  //! the interference.
    const Intf_SectionLine& LineValue (const int Index) const;
  
  //! Gives the number of zones of tangence in the interference.
    int NbTangentZones() const;
  
  //! Gives  the zone of  tangence at address   Index in the
  //! interference.
    const Intf_TangentZone& ZoneValue (const int Index) const;
  
  //! Gives the tolerance used for the calculation.
    double GetTolerance() const;
  
  //! Tests if the polylines of  intersection or the zones of
  //! tangence contain the point of intersection <ThePnt>.
  mobiusGeom_EXPORT bool Contains (const Intf_SectionPoint& ThePnt) const;
  
  //! Inserts a new zone of tangence in  the current list of
  //! tangent zones of  the interference  and  returns  True
  //! when done.
  mobiusGeom_EXPORT bool Insert (const Intf_TangentZone& TheZone);
  
  //! Insert a new segment of intersection in the current  list of
  //! polylines of intersection of the interference.
  mobiusGeom_EXPORT void Insert (const Intf_SectionPoint& pdeb, const Intf_SectionPoint& pfin);
  
  mobiusGeom_EXPORT void Dump() const;




protected:

  //! Empty constructor
  mobiusGeom_EXPORT Intf_Interference(const bool Self);
  
  //! Destructor is protected, for safer inheritance
  ~Intf_Interference () {}

  //! Only one argument for the intersection.
  mobiusGeom_EXPORT void SelfInterference (const bool Self);


  Intf_SeqOfSectionPoint mySPoins;
  Intf_SeqOfSectionLine mySLines;
  Intf_SeqOfTangentZone myTZones;
  bool SelfIntf;
  double Tolerance;


private:





};


//=======================================================================
// Return the number of sections points in an interference.
//=======================================================================

inline int Intf_Interference::NbSectionPoints () const
{
  return mySPoins.Length();
}

//=======================================================================
// Give the section point of range Index in the interference.
//=======================================================================

inline const Intf_SectionPoint& Intf_Interference::PntValue (const int Index) const
{
  return mySPoins(Index);
}

//=======================================================================
// Return the number of sections lines in an interference.
//=======================================================================

inline int Intf_Interference::NbSectionLines () const
{
  return mySLines.Length();
}

//=======================================================================
// Give the section line of range Index in the interference.
//=======================================================================

inline const Intf_SectionLine& Intf_Interference::LineValue (const int Index) const
{
  return mySLines(Index);
}

//=======================================================================
// Return the number of sections TangentZones in an interference.
//=======================================================================

inline int Intf_Interference::NbTangentZones () const
{
  return myTZones.Length();
}

//=======================================================================
// Give the tangentzone of range Index in the interference.
//=======================================================================

inline const Intf_TangentZone& Intf_Interference::ZoneValue (const int Index) const
{
  return myTZones(Index);
}

//=======================================================================
//function : GetTolerance
//purpose  : 
//=======================================================================

inline double Intf_Interference::GetTolerance () const
{
  return Tolerance;
}

#endif // _Intf_Interference_HeaderFile
