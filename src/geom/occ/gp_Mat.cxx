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

#include <mobius/gp_Mat.hxx>

#include <mobius/gp_GTrsf.hxx>
#include <mobius/gp_Trsf.hxx>
#include <mobius/gp_XYZ.hxx>

using namespace mobius::occ;

// =======================================================================
// function : gp_Mat
// purpose  :
// =======================================================================
gp_Mat::gp_Mat (const gp_XYZ& theCol1,
		            const gp_XYZ& theCol2,
		            const gp_XYZ& theCol3)
{
  myMat[0][0] = theCol1.X(); myMat[1][0] = theCol1.Y(); myMat[2][0] = theCol1.Z();
  myMat[0][1] = theCol2.X(); myMat[1][1] = theCol2.Y(); myMat[2][1] = theCol2.Z();
  myMat[0][2] = theCol3.X(); myMat[1][2] = theCol3.Y(); myMat[2][2] = theCol3.Z();
}

// =======================================================================
// function : SetCol
// purpose  :
// =======================================================================
void gp_Mat::SetCol (const int theCol,
		                 const gp_XYZ& theValue)
{
  if      (theCol == 1)
  {
    myMat[0][0] = theValue.X(); myMat[1][0] = theValue.Y(); myMat[2][0] = theValue.Z();
  }
  else if (theCol == 2)
  {
    myMat[0][1] = theValue.X(); myMat[1][1] = theValue.Y(); myMat[2][1] = theValue.Z();
  }
  else
  {
    myMat[0][2] = theValue.X(); myMat[1][2] = theValue.Y(); myMat[2][2] = theValue.Z();
  }
}

// =======================================================================
// function : SetCols
// purpose  :
// =======================================================================
void gp_Mat::SetCols (const gp_XYZ& theCol1,
		                  const gp_XYZ& theCol2,
		                  const gp_XYZ& theCol3)
{
  myMat[0][0] = theCol1.X(); myMat[1][0] = theCol1.Y(); myMat[2][0] = theCol1.Z();
  myMat[0][1] = theCol2.X(); myMat[1][1] = theCol2.Y(); myMat[2][1] = theCol2.Z();
  myMat[0][2] = theCol3.X(); myMat[1][2] = theCol3.Y(); myMat[2][2] = theCol3.Z();
}

// =======================================================================
// function : SetCross
// purpose  :
// =======================================================================
void gp_Mat::SetCross (const gp_XYZ& theRef)
{
  const double X = theRef.X();
  const double Y = theRef.Y();
  const double Z = theRef.Z();
  myMat[0][0] = myMat[1][1] = myMat[2][2] = 0.0;
  myMat[0][1] = -Z;
  myMat[0][2] =  Y;
  myMat[1][2] = -X;
  myMat[1][0] =  Z;
  myMat[2][0] = -Y;
  myMat[2][1] =  X;
}

// =======================================================================
// function : SetDot
// purpose  :
// =======================================================================
void gp_Mat::SetDot (const gp_XYZ& theRef)
{
  const double X = theRef.X();
  const double Y = theRef.Y();
  const double Z = theRef.Z();
  myMat[0][0] = X * X;
  myMat[1][1] = Y * Y;
  myMat[2][2] = Z * Z;
  myMat[0][1] = X * Y;
  myMat[0][2] = X * Z;
  myMat[1][2] = Y * Z;
  myMat[1][0] = myMat[0][1];
  myMat[2][0] = myMat[0][2];
  myMat[2][1] = myMat[1][2];
}

// =======================================================================
// function : SetRotation
// purpose  :
// =======================================================================
void gp_Mat::SetRotation (const gp_XYZ& theAxis,
			                    const double theAng)
{
  //    Rot = I + sin(Ang) * M + (1. - cos(Ang)) * M*M
  //    avec  M . XYZ = Axis ^ XYZ
  const gp_XYZ aV = theAxis.Normalized();
  SetCross (aV);
  Multiply (sin(theAng));
  gp_Mat aTemp;
  aTemp.SetScale (1.0);
  Add (aTemp);
  const double A = aV.X();
  const double B = aV.Y();
  const double C = aV.Z();
  aTemp.SetRow (1, gp_XYZ(- C*C - B*B,      A*B,           A*C     ));
  aTemp.SetRow (2, gp_XYZ(     A*B,      -A*A - C*C,        B*C    ));
  aTemp.SetRow (3, gp_XYZ(     A*C,          B*C,       - A*A - B*B));
  aTemp.Multiply (1.0 - cos(theAng));
  Add (aTemp);
}

// =======================================================================
// function : SetRow
// purpose  :
// =======================================================================
void gp_Mat::SetRow (const int theRow,
		                 const gp_XYZ& theValue)
{
  if      (theRow == 1)
  {
    myMat[0][0] = theValue.X(); myMat[0][1] = theValue.Y(); myMat[0][2] = theValue.Z();
  }
  else if (theRow == 2)
  {
    myMat[1][0] = theValue.X(); myMat[1][1] = theValue.Y(); myMat[1][2] = theValue.Z();
  }
  else
  {
    myMat[2][0] = theValue.X(); myMat[2][1] = theValue.Y(); myMat[2][2] = theValue.Z();
  }
}

// =======================================================================
// function : SetRows
// purpose  :
// =======================================================================
void gp_Mat::SetRows (const gp_XYZ& theRow1,
		                  const gp_XYZ& theRow2,
		                  const gp_XYZ& theRow3)
{
  myMat[0][0] = theRow1.X(); myMat[0][1] = theRow1.Y(); myMat[0][2] = theRow1.Z();
  myMat[1][0] = theRow2.X(); myMat[1][1] = theRow2.Y(); myMat[1][2] = theRow2.Z();
  myMat[2][0] = theRow3.X(); myMat[2][1] = theRow3.Y(); myMat[2][2] = theRow3.Z();
}

// =======================================================================
// function : Column
// purpose  :
// =======================================================================
gp_XYZ gp_Mat::Column (const int theCol) const
{
  if (theCol == 1) return gp_XYZ (myMat[0][0], myMat[1][0], myMat[2][0]);
  if (theCol == 2) return gp_XYZ (myMat[0][1], myMat[1][1], myMat[2][1]);
  return gp_XYZ (myMat[0][2], myMat[1][2], myMat[2][2]);
}

// =======================================================================
// function : Diagonal
// purpose  :
// =======================================================================
gp_XYZ gp_Mat::Diagonal() const
{
  return gp_XYZ (myMat[0][0], myMat[1][1], myMat[2][2]);
}

// =======================================================================
// function : Row
// purpose  :
// =======================================================================
gp_XYZ gp_Mat::Row (const int theRow) const
{
  if (theRow == 1) return gp_XYZ (myMat[0][0], myMat[0][1], myMat[0][2]);
  if (theRow == 2) return gp_XYZ (myMat[1][0], myMat[1][1], myMat[1][2]);
  return gp_XYZ (myMat[2][0], myMat[2][1], myMat[2][2]);
}

// =======================================================================
// function : Invert
// purpose  :
// =======================================================================
void gp_Mat::Invert()
{ 
  double aNewMat[3][3];
  // calcul de  la transposee de la commatrice
  aNewMat[0][0] =   myMat[1][1] * myMat[2][2] - myMat[1][2] * myMat[2][1];
  aNewMat[1][0] = -(myMat[1][0] * myMat[2][2] - myMat[2][0] * myMat[1][2]);
  aNewMat[2][0] =   myMat[1][0] * myMat[2][1] - myMat[2][0] * myMat[1][1];
  aNewMat[0][1] = -(myMat[0][1] * myMat[2][2] - myMat[2][1] * myMat[0][2]);
  aNewMat[1][1] =   myMat[0][0] * myMat[2][2] - myMat[2][0] * myMat[0][2];
  aNewMat[2][1] = -(myMat[0][0] * myMat[2][1] - myMat[2][0] * myMat[0][1]);
  aNewMat[0][2] =   myMat[0][1] * myMat[1][2] - myMat[1][1] * myMat[0][2];
  aNewMat[1][2] = -(myMat[0][0] * myMat[1][2] - myMat[1][0] * myMat[0][2]);
  aNewMat[2][2] =   myMat[0][0] * myMat[1][1] - myMat[0][1] * myMat[1][0];
  double aDet =  myMat[0][0] * aNewMat[0][0] + myMat[0][1]* aNewMat[1][0] + myMat[0][2] * aNewMat[2][0];
  double aVal = aDet;
  if (aVal < 0) aVal = -aVal;
  aDet = 1.0e0 / aDet;
  myMat[0][0] = aNewMat[0][0];
  myMat[1][0] = aNewMat[1][0];
  myMat[2][0] = aNewMat[2][0];
  myMat[0][1] = aNewMat[0][1];
  myMat[1][1] = aNewMat[1][1];
  myMat[2][1] = aNewMat[2][1];
  myMat[0][2] = aNewMat[0][2];
  myMat[1][2] = aNewMat[1][2];
  myMat[2][2] = aNewMat[2][2];
  Multiply (aDet);
}

// =======================================================================
// function : Inverted
// purpose  :
// =======================================================================
gp_Mat gp_Mat::Inverted() const
{ 
  gp_Mat aNewMat;
  // calcul de  la transposee de la commatrice
  aNewMat.myMat[0][0] =   myMat[1][1] * myMat[2][2] - myMat[1][2] * myMat[2][1];
  aNewMat.myMat[1][0] = -(myMat[1][0] * myMat[2][2] - myMat[2][0] * myMat[1][2]);
  aNewMat.myMat[2][0] =   myMat[1][0] * myMat[2][1] - myMat[2][0] * myMat[1][1];
  aNewMat.myMat[0][1] = -(myMat[0][1] * myMat[2][2] - myMat[2][1] * myMat[0][2]);
  aNewMat.myMat[1][1] =   myMat[0][0] * myMat[2][2] - myMat[2][0] * myMat[0][2];
  aNewMat.myMat[2][1] = -(myMat[0][0] * myMat[2][1] - myMat[2][0] * myMat[0][1]);
  aNewMat.myMat[0][2] =   myMat[0][1] * myMat[1][2] - myMat[1][1] * myMat[0][2];
  aNewMat.myMat[1][2] = -(myMat[0][0] * myMat[1][2] - myMat[1][0] * myMat[0][2]);
  aNewMat.myMat[2][2] =   myMat[0][0] * myMat[1][1] - myMat[0][1] * myMat[1][0];
  double aDet =  myMat[0][0] * aNewMat.myMat[0][0] + myMat[0][1]* aNewMat.myMat[1][0] + myMat[0][2] * aNewMat.myMat[2][0];
  double aVal =  aDet;
  if (aVal < 0) aVal = -aVal;
  aDet = 1.0e0 / aDet;
  aNewMat.Multiply (aDet);
  return aNewMat;
}

// =======================================================================
// function : Power
// purpose  :
// =======================================================================
void gp_Mat::Power (const int theN)
{
  if (theN == 1) { }
  else if (theN == 0)  { SetIdentity(); }
  else if (theN == -1) { Invert(); }
  else {
    if (theN < 0) { Invert(); }
    int Npower = theN;
    if (Npower < 0) Npower = - Npower;
    Npower--;
    gp_Mat aTemp = *this;
    for(;;) {
      if (IsOdd(Npower)) Multiply (aTemp);
      if (Npower == 1)   break; 
      aTemp.Multiply (aTemp);
      Npower>>=1;
    }
  }
}
