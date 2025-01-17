// Copyright (c) 1998-1999 Matra Datavision
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

#include <mobius/Standard_ShortReal.hxx>
#include <mobius/Standard_RangeError.hxx>
#include <mobius/Standard_NullValue.hxx>
#include <mobius/Standard_Stream.hxx>

using namespace mobius::occ;

//============================================================================
// function : HashCode
// purpose  :
//============================================================================
int HashCode (const Standard_ShortReal theShortReal, const int theUpperBound)
{
  if (theUpperBound < 1)
  {
    throw Standard_RangeError ("Try to apply HashCode method with negative or null argument.");
  }
  union
  {
    Standard_ShortReal R;
    int   I;
  } U;
  U.R = theShortReal;

  return HashCode (U.I, theUpperBound);
}
