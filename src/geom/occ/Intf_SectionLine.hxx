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

#ifndef _Intf_SectionLine_HeaderFile
#define _Intf_SectionLine_HeaderFile

#include <mobius/Intf_SeqOfSectionPoint.hxx>

namespace mobius {
namespace occ {

//! Describe    a  polyline  of   intersection  between two
//! polyhedra as a sequence of points of intersection.
class Intf_SectionLine
{
public:

  //! Returns number of points in this SectionLine.
  int NumberOfPoints() const;

  //! Gives the point of intersection of  address <Index>  in the
  //! SectionLine.
  mobiusGeom_EXPORT const Intf_SectionPoint& GetPoint (const int Index) const;
  
  //! Returns True if the SectionLine is closed.
  mobiusGeom_EXPORT bool IsClosed() const;
  
  //! Returns True if ThePI is in the SectionLine <me>.
  mobiusGeom_EXPORT bool Contains (const Intf_SectionPoint& ThePI) const;
  
  //! Checks if <ThePI>  is an end of  the SectionLine. Returns 1
  //! for the beginning, 2 for the end, otherwise 0.
  mobiusGeom_EXPORT int IsEnd (const Intf_SectionPoint& ThePI) const;
  
  //! Compares two SectionLines.
  mobiusGeom_EXPORT bool IsEqual (const Intf_SectionLine& Other) const;
bool operator == (const Intf_SectionLine& Other) const
{
  return IsEqual(Other);
}
  
  //! Constructs an empty SectionLine.
  mobiusGeom_EXPORT Intf_SectionLine();
  
  //! Copies a SectionLine.
  mobiusGeom_EXPORT Intf_SectionLine(const Intf_SectionLine& Other);
  
  //! Assignment
  Intf_SectionLine& operator= (const Intf_SectionLine& theOther)
  {
    //closed = theOther.closed; // not copied as in copy constructor
    myPoints = theOther.myPoints;
    return *this;
  }

  //! Adds a point at the end of the SectionLine.
  mobiusGeom_EXPORT void Append (const Intf_SectionPoint& Pi);
  
  //! Concatenates   the SectionLine  <LS>  at  the  end  of  the
  //! SectionLine <me>.
  mobiusGeom_EXPORT void Append (Intf_SectionLine& LS);
  
  //! Adds a point to the beginning of the SectionLine <me>.
  mobiusGeom_EXPORT void Prepend (const Intf_SectionPoint& Pi);

  //! Concatenates a SectionLine  <LS>  at the  beginning  of the
  //! SectionLine <me>.
  mobiusGeom_EXPORT void Prepend (Intf_SectionLine& LS);

  //! Reverses the order of the elements of the SectionLine.
  mobiusGeom_EXPORT void Reverse();

  //! Closes the SectionLine.
  mobiusGeom_EXPORT void Close();

  mobiusGeom_EXPORT void Dump (const int Indent) const;

private:

  Intf_SeqOfSectionPoint myPoints;
  bool closed;

};

inline int Intf_SectionLine::NumberOfPoints() const
{
  return myPoints.Length();
}

}
}

#endif // _Intf_SectionLine_HeaderFile
