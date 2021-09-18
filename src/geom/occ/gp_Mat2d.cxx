// Copyright (c) 1995-1999 Matra Datavision
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

#include <mobius/gp_Mat2d.hxx>

#include <mobius/gp_GTrsf2d.hxx>
#include <mobius/gp_Trsf2d.hxx>
#include <mobius/gp_XY.hxx>

using namespace mobius;

// =======================================================================
// function : gp_Mat2d
// purpose  :
// =======================================================================
gp_Mat2d::gp_Mat2d (const gp_XY& theCol1, const gp_XY& theCol2)
{
  myMat[0][0] = theCol1.X(); myMat[1][0] = theCol1.Y();
  myMat[0][1] = theCol2.X(); myMat[1][1] = theCol2.Y();
}

// =======================================================================
// function : SetCol
// purpose  :
// =======================================================================
void gp_Mat2d::SetCol (const int theCol,
                       const gp_XY& theValue)
{
  if  (theCol == 1)
  {
    myMat[0][0] = theValue.X();
    myMat[1][0] = theValue.Y();
  }
  else
  {
    myMat[0][1] = theValue.X();
    myMat[1][1] = theValue.Y();
  }
}

// =======================================================================
// function : SetCols
// purpose  :
// =======================================================================
void gp_Mat2d::SetCols (const gp_XY& theCol1,
                        const gp_XY& theCol2)
{
  myMat[0][0] = theCol1.X(); myMat[1][0] = theCol1.Y();
  myMat[0][1] = theCol2.X(); myMat[1][1] = theCol2.Y();
}

// =======================================================================
// function : SetRow
// purpose  :
// =======================================================================
void gp_Mat2d::SetRow (const int theRow, const gp_XY& theValue)
{
  if (theRow == 1)
  {
    myMat[0][0] = theValue.X();
    myMat[0][1] = theValue.Y();
  }
  else
  {
    myMat[1][0] = theValue.X();
    myMat[1][1] = theValue.Y();
  }
}

// =======================================================================
// function : SetRows
// purpose  :
// =======================================================================
void gp_Mat2d::SetRows (const gp_XY& theRow1, const gp_XY& theRow2)
{
  myMat[0][0] = theRow1.X(); myMat[0][1] = theRow1.Y();
  myMat[1][0] = theRow2.X(); myMat[1][1] = theRow2.Y();
}

// =======================================================================
// function : Column
// purpose  :
// =======================================================================
gp_XY gp_Mat2d::Column (const int theCol) const
{
  if (theCol == 1)
  {
    return gp_XY (myMat[0][0], myMat[1][0]);
  }
  return gp_XY (myMat[0][1], myMat[1][1]);
}

// =======================================================================
// function : Diagonal
// purpose  :
// =======================================================================
gp_XY gp_Mat2d::Diagonal() const
{ 
  return gp_XY (myMat[0][0], myMat[1][1]);
}

// =======================================================================
// function : Row
// purpose  :
// =======================================================================
gp_XY gp_Mat2d::Row (const int theRow) const
{
  if (theRow == 1)
  {
    return gp_XY (myMat[0][0], myMat[0][1]);
  }
  return gp_XY (myMat[1][0], myMat[1][1]);
}

// =======================================================================
// function : Invert
// purpose  :
// =======================================================================
void gp_Mat2d::Invert()
{
  double aNewMat[2][2];
  aNewMat[0][0] =  myMat[1][1];
  aNewMat[0][1] = -myMat[0][1];
  aNewMat[1][0] = -myMat[1][0];
  aNewMat[1][1] =  myMat[0][0];
  double aDet = aNewMat[0][0] * aNewMat[1][1] - aNewMat[0][1] * aNewMat[1][0];
  double val = aDet;
  if (val < 0) val = - val;
  aDet = 1.0 / aDet;
  myMat[0][0] = aNewMat[0][0] * aDet;
  myMat[1][0] = aNewMat[1][0] * aDet;
  myMat[0][1] = aNewMat[0][1] * aDet;
  myMat[1][1] = aNewMat[1][1] * aDet;
}

// =======================================================================
// function : Power
// purpose  :
// =======================================================================
void gp_Mat2d::Power (const int theN)
{
  if      (theN ==  1) { }
  else if (theN ==  0) { SetIdentity(); }
  else if (theN == -1) { Invert(); }
  else {
    if (theN < 0) Invert();
    int Npower = theN;
    if (Npower < 0) Npower = - Npower;
    Npower--;
    gp_Mat2d aTemp = *this;
    for(;;) {
      if (IsOdd(Npower)) Multiply (aTemp);
      if (Npower == 1)   break;
      aTemp.Multiply (aTemp);
      Npower = Npower/2;
    }
  }
}
