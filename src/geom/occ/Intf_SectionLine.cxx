// Created on: 1991-06-21
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


#include <mobius/Intf_SectionLine.hxx>
#include <mobius/Intf_SectionPoint.hxx>

using namespace mobius::occ;

//=======================================================================
//function : Intf_SectionLine
//purpose  : Construct
//=======================================================================
Intf_SectionLine::Intf_SectionLine ()
     : closed(false)
{}

//=======================================================================
//function : Intf_SectionLine
//purpose  : Copy
//=======================================================================

Intf_SectionLine::Intf_SectionLine (const Intf_SectionLine& Other)
     : closed(false)
{
  myPoints=Other.myPoints;
}


//=======================================================================
//function : Append
//purpose  : 
//=======================================================================

void Intf_SectionLine::Append (const Intf_SectionPoint& Pi)
{
  myPoints.Append(Pi);
}

//=======================================================================
//function : Append
//purpose  : 
//=======================================================================

void Intf_SectionLine::Append (Intf_SectionLine& LS)
{
  myPoints.Append(LS.myPoints);
}

//=======================================================================
//function : Prepend
//purpose  : 
//=======================================================================

void Intf_SectionLine::Prepend (const Intf_SectionPoint& Pi)
{
  myPoints.Prepend(Pi);
}

//=======================================================================
//function : Prepend
//purpose  : 
//=======================================================================

void Intf_SectionLine::Prepend (Intf_SectionLine& LS)
{
  myPoints.Prepend(LS.myPoints);
}

//=======================================================================
//function : Reverse
//purpose  : 
//=======================================================================

void Intf_SectionLine::Reverse ()
{
  myPoints.Reverse();
}

//=======================================================================
//function : Close
//purpose  : 
//=======================================================================

void Intf_SectionLine::Close ()
{
  closed=true;
}


//=======================================================================
//function : GetPoint
//purpose  : 
//=======================================================================

const Intf_SectionPoint& Intf_SectionLine::GetPoint
  (const int index) const
{
  return myPoints.Value(index);
}


//=======================================================================
//function : IsClosed
//purpose  : 
//=======================================================================

bool Intf_SectionLine::IsClosed () const
{
//return closed;
// On ne peut fermer une ligne de section inseree dans une liste car
// la fonction Value() copie l'element avant d'appeler la fonction.
// C'est donc la copie qui est modifiee.
// Pour la sequence myPoints on a un pointeur donc le pb ne se pose pas.

  return (myPoints.First()==myPoints.Last());
}

//=======================================================================
//function : Contains
//purpose  : 
//=======================================================================

bool Intf_SectionLine::Contains
 (const Intf_SectionPoint& ThePI) const
{
  for (int i = 1; i <= myPoints.Length();i++)
    if (ThePI.IsEqual(myPoints(i))) return true;
  return false;
}


//=======================================================================
//function : IsEnd
//purpose  : 
//=======================================================================

int Intf_SectionLine::IsEnd 
  (const Intf_SectionPoint& ThePI) const
{
  if (myPoints.First().IsEqual(ThePI)) return 1;
  if (myPoints.Last().IsEqual(ThePI)) return myPoints.Length();
  return 0;
}


//=======================================================================
//function : IsEqual
//purpose  : 
//=======================================================================

bool Intf_SectionLine::IsEqual 
  (const Intf_SectionLine& Other) const
{
  if (myPoints.Length() != Other.myPoints.Length())
    return false;
  for (int i = 1; i <= myPoints.Length(); i++)
    if (!myPoints(i).IsEqual(Other.myPoints(i))) return false;
  return true;
}

//=======================================================================
//function : Dump
//purpose  : 
//=======================================================================

void Intf_SectionLine::Dump
 (const int Indent) const
{
  for (int id=0; id<Indent; id++) std::cout << " ";
  std::cout << "LS ";
  if (IsClosed()) std::cout << "Closed :" << std::endl;
  else std::cout << "Open :" << std::endl;
  for (int p=1; p<=myPoints.Length(); p++) {
    myPoints.Value(p).Dump(Indent+2);
  }
}
