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

// JCV 08/01/91 Modifs introduction des classes XY, Mat2d dans le package gp

#define No_Standard_OutOfRange

#include <mobius/gp_Pnt2d.hxx>

#include <mobius/gp_Ax2d.hxx>
#include <mobius/gp_Trsf2d.hxx>
#include <mobius/gp_Vec2d.hxx>
#include <mobius/gp_XY.hxx>

using namespace mobius::occ;

//=======================================================================
//function : Distance
// purpose :
//=======================================================================
double gp_Pnt2d::Distance (const gp_Pnt2d& theOther) const
{
  const gp_XY& aXY = theOther.coord;
  double aX = coord.X() - aXY.X();
  double aY = coord.Y() - aXY.Y();
  return sqrt (aX * aX + aY * aY);
}

//=======================================================================
//function : SquareDistance
// purpose :
//=======================================================================
double gp_Pnt2d::SquareDistance (const gp_Pnt2d& theOther) const
{
  const gp_XY& aXY = theOther.coord;
  double aX = coord.X() - aXY.X();
  double aY = coord.Y() - aXY.Y();
  return (aX * aX + aY * aY);
}

//=======================================================================
//function : Rotate
// purpose :
//=======================================================================
void gp_Pnt2d::Rotate (const gp_Pnt2d& theP, const double theAng)
{
  gp_Trsf2d aT;
  aT.SetRotation (theP, theAng);
  aT.Transforms (coord);
}

//=======================================================================
//function : Scale
// purpose :
//=======================================================================
void gp_Pnt2d::Scale (const gp_Pnt2d& theP, const double theS)
{
  gp_XY aXY = theP.coord;
  aXY.Multiply (1.0 - theS);
  coord.Multiply (theS);
  coord.Add (aXY);
}

//=======================================================================
//function : Translate
// purpose :
//=======================================================================
void gp_Pnt2d::Translate(const gp_Vec2d& theV)
{
  coord.Add (theV.XY());
}

//=======================================================================
//function : Translated
// purpose :
//=======================================================================
gp_Pnt2d gp_Pnt2d::Translated (const gp_Vec2d& theV) const
{
  gp_Pnt2d aP = *this;
  aP.coord.Add (theV.XY());
  return aP;
}

void gp_Pnt2d::Transform (const gp_Trsf2d& T)
{
  if (T.Form () == gp_Identity) { }
  else if (T.Form () == gp_Translation)
    { coord.Add (T.TranslationPart ()); }
  else if (T.Form () == gp_Scale) {
    coord.Multiply (T.ScaleFactor ());
    coord.Add      (T.TranslationPart ());
  }
  else if (T.Form () == gp_PntMirror) {
    coord.Reverse ();
    coord.Add     (T.TranslationPart ());
  }
  else { T.Transforms(coord); }
}

void gp_Pnt2d::Mirror (const gp_Pnt2d& P)
{
  coord.Reverse ();
  gp_XY XY = P.coord;
  XY.Multiply (2.0);
  coord.Add (XY);
}

gp_Pnt2d gp_Pnt2d::Mirrored (const gp_Pnt2d& P) const
{
  gp_Pnt2d Pres = *this;
  Pres.Mirror (P);
  return Pres;
}

void gp_Pnt2d::Mirror (const gp_Ax2d& A)
{
  gp_Trsf2d T;
  T.SetMirror  (A);
  T.Transforms (coord);
}

gp_Pnt2d gp_Pnt2d::Mirrored (const gp_Ax2d& A) const
{
  gp_Pnt2d P = *this;
  P.Mirror (A);
  return P;
}
