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

// JCV 30/08/90 Modif passage version C++ 2.0 sur Sun
// JCV 01/10/90 Changement de nom du package vgeom -> gp
// JCV 07/12/90 Modifs introduction des classes XYZ, Mat dans le package gp

#define No_Standard_OutOfRange

#include <mobius/gp_Pnt.hxx>

#include <mobius/gp_Ax1.hxx>
#include <mobius/gp_Ax2.hxx>
#include <mobius/gp_Trsf.hxx>
#include <mobius/gp_Vec.hxx>
#include <mobius/gp_XYZ.hxx>

using namespace mobius::occ;

//=======================================================================
//function : Distance
// purpose :
//=======================================================================
double gp_Pnt::Distance (const gp_Pnt& theOther) const
{
  double aD=0,aDD;
  const gp_XYZ& aXYZ = theOther.coord;
  aDD = coord.X(); aDD -= aXYZ.X(); aDD *= aDD; aD += aDD;
  aDD = coord.Y(); aDD -= aXYZ.Y(); aDD *= aDD; aD += aDD;
  aDD = coord.Z(); aDD -= aXYZ.Z(); aDD *= aDD; aD += aDD;
  return sqrt (aD);
}

//=======================================================================
//function : SquareDistance
// purpose :
//=======================================================================
double gp_Pnt::SquareDistance (const gp_Pnt& theOther) const
{
  double aD=0, aDD;
  const gp_XYZ& XYZ = theOther.coord;
  aDD = coord.X(); aDD -= XYZ.X(); aDD *= aDD; aD += aDD;
  aDD = coord.Y(); aDD -= XYZ.Y(); aDD *= aDD; aD += aDD;
  aDD = coord.Z(); aDD -= XYZ.Z(); aDD *= aDD; aD += aDD;
  return aD;
}

//=======================================================================
//function : Rotate
// purpose :
//=======================================================================
void gp_Pnt::Rotate (const gp_Ax1& theA1, const double theAng)
{
  gp_Trsf aT;
  aT.SetRotation (theA1, theAng);
  aT.Transforms (coord);
}

//=======================================================================
//function : Scale
// purpose :
//=======================================================================
void gp_Pnt::Scale (const gp_Pnt& theP, const double theS)
{
  gp_XYZ aXYZ = theP.coord;
  aXYZ.Multiply (1.0 - theS);
  coord.Multiply (theS);
  coord.Add (aXYZ);
}

//=======================================================================
//function : Translate
// purpose :
//=======================================================================
void gp_Pnt::Translate(const gp_Vec& theV)
{
  coord.Add (theV.XYZ());
}

//=======================================================================
//function : Translated
// purpose :
//=======================================================================
gp_Pnt gp_Pnt::Translated (const gp_Vec& theV) const
{
  gp_Pnt aP = *this;
  aP.coord.Add (theV.XYZ());
  return aP;
}

void gp_Pnt::Transform (const gp_Trsf& T)
{
  if (T.Form() == gp_Identity) { }
  else if (T.Form() == gp_Translation) { coord.Add (T.TranslationPart ()); }
  else if (T.Form() == gp_Scale) {
    coord.Multiply (T.ScaleFactor ());
    coord.Add      (T.TranslationPart ());
  }
  else if(T.Form() == gp_PntMirror) {
    coord.Reverse ();
    coord.Add     (T.TranslationPart ());
  }
  else { T.Transforms(coord); }
}

void gp_Pnt::Mirror (const gp_Pnt& P)
{
  coord.Reverse ();
  gp_XYZ XYZ = P.coord;
  XYZ.Multiply (2.0);
  coord.Add      (XYZ);
}

gp_Pnt gp_Pnt::Mirrored (const gp_Pnt& P) const
{
  gp_Pnt Pres = *this;
  Pres.Mirror (P);
  return Pres;
}

void gp_Pnt::Mirror (const gp_Ax1& A1)
{
  gp_Trsf T;
  T.SetMirror  (A1);
  T.Transforms (coord);
}

gp_Pnt gp_Pnt::Mirrored (const gp_Ax1& A1) const
{
  gp_Pnt P = *this;
  P.Mirror (A1);
  return P;
}

void gp_Pnt::Mirror (const gp_Ax2& A2)
{
  gp_Trsf T;
  T.SetMirror  (A2);
  T.Transforms (coord);
}

gp_Pnt gp_Pnt::Mirrored (const gp_Ax2& A2) const
{
  gp_Pnt P = *this;
  P.Mirror (A2);
  return P;
}
