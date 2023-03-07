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

#include <mobius/gp_GTrsf2d.hxx>

#include <mobius/gp_Ax2d.hxx>
#include <mobius/gp_Mat2d.hxx>
#include <mobius/gp_Trsf2d.hxx>
#include <mobius/gp_XY.hxx>
#include <mobius/occMathDefs.hxx>
#include <mobius/Standard_Integer.hxx>

using namespace mobius::occ;

//=======================================================================
//function : SetTrsf2d
// purpose :
//=======================================================================
void gp_GTrsf2d::SetTrsf2d (const gp_Trsf2d& theT)
{
  shape = theT.shape;
  matrix = theT.matrix;
  loc = theT.loc;
  scale = theT.scale;
}

//=======================================================================
//function : gp_GTrsf2d
// purpose :
//=======================================================================
gp_GTrsf2d::gp_GTrsf2d (const gp_Trsf2d& theT)
{
  shape = theT.shape;
  matrix = theT.matrix;
  loc = theT.loc;
  scale = theT.scale;
}

//=======================================================================
//function : SetValue
// purpose :
//=======================================================================
void gp_GTrsf2d::SetValue (const int theRow,
                           const int theCol,
                           const double theValue)
{
  if (theCol == 3)
  {
    loc.SetCoord (theRow, theValue);
  }
  else
  {
    matrix.SetValue (theRow, theCol, theValue);
  }
  shape = gp_Other;
}

//=======================================================================
//function : Value
// purpose :
//=======================================================================
double gp_GTrsf2d::Value (const int theRow,
                          const int theCol) const
{
  if (theCol == 3)
  {
    return loc.Coord (theRow);
  }
  if (shape == gp_Other)
  {
    return matrix.Value (theRow, theCol);
  }
  return scale * matrix.Value (theRow, theCol);
}

//=======================================================================
//function : Transforms
// purpose :
//=======================================================================
void gp_GTrsf2d::Transforms (gp_XY& theCoord) const
{
  theCoord.Multiply (matrix);
  if (!(shape == gp_Other) && !(scale == 1.0))
  {
    theCoord.Multiply (scale);
  }
  theCoord.Add (loc);
}

//=======================================================================
//function : Transforms
// purpose :
//=======================================================================
void gp_GTrsf2d::Transforms (double& theX,
                             double& theY) const
{
  gp_XY aDoublet(theX, theY);
  aDoublet.Multiply (matrix);
  if (!(shape == gp_Other) && !(scale == 1.0))
  {
    aDoublet.Multiply (scale);
  }
  aDoublet.Add (loc);
  aDoublet.Coord (theX, theY);
}

void gp_GTrsf2d::SetAffinity (const gp_Ax2d& A,
			      const double Ratio)
{
  shape = gp_Other;
  scale = 0.0;
  double a = A.Direction().X();
  double b = A.Direction().Y();
  matrix.SetValue (1, 1, (1.0 - Ratio)*a*a + Ratio);
  matrix.SetValue (2, 2, (1.0 - Ratio)*b*b + Ratio);
  matrix.SetValue (1, 2, (1.0 - Ratio)*a*b);
  matrix.SetValue (2, 1, matrix.Value (1, 2));
  loc = A.Location().XY();
  loc.Reverse();
  loc.Multiply (matrix);
  loc.Add (A.Location().XY());
}

void gp_GTrsf2d::SetTranslationPart (const gp_XY& Coord)
{
  loc = Coord;
  if (Form() == gp_CompoundTrsf || Form() == gp_Other || 
      Form() == gp_Translation) { }
  else if (Form() == gp_Identity) { shape = gp_Translation; }
  else { shape = gp_CompoundTrsf; }
}

void gp_GTrsf2d::Invert ()
{
  if (shape == gp_Other) {
    matrix.Invert();
    loc.Multiply (matrix);
    loc.Reverse();
  }
  else {
    gp_Trsf2d T = Trsf2d ();
    T.Invert ();
    SetTrsf2d (T);
  }
}

void gp_GTrsf2d::Multiply (const gp_GTrsf2d& T)
{
  if (Form() == gp_Other || T.Form() == gp_Other) {
    shape = gp_Other;
    loc.Add (T.loc.Multiplied (matrix));
    matrix.Multiply(T.matrix);
  }
  else {
    gp_Trsf2d T1 = Trsf2d();
    gp_Trsf2d T2 = T.Trsf2d();
    T1.Multiply(T2);
    matrix = T1.matrix;
    loc = T1.loc;
    scale = T1.scale;
    shape = T1.shape;
  }
}

void gp_GTrsf2d::Power (const int N)
{
  if (N == 0)  {
    scale = 1.0;
    shape = gp_Identity;
    matrix.SetIdentity();
    loc = gp_XY (0., 0.);
  }
  else if (N == 1) { }
  else if (N == -1) { Invert(); }
  else {
    if (N < 0) { Invert(); }
    if (shape == gp_Other) {
      int Npower = N;
      if (Npower < 0) Npower = - Npower;
      Npower--;
      gp_XY Temploc = loc;
//      double Tempscale = scale;
      gp_Mat2d Tempmatrix (matrix);
      for(;;) {
	if (IsOdd(Npower)) {
	  loc.Add (Temploc.Multiplied (matrix));
	  matrix.Multiply (Tempmatrix);
	}
	if (Npower == 1) { break; }
	Temploc.Add (Temploc.Multiplied (Tempmatrix));
	Tempmatrix.Multiply (Tempmatrix);
	Npower = Npower/2;
      }
    }
    else {
      gp_Trsf2d T = Trsf2d ();
      T.Power (N);
      SetTrsf2d (T);
    }
  }
}

void gp_GTrsf2d::PreMultiply (const gp_GTrsf2d& T)
{
  if (Form() == gp_Other || T.Form() == gp_Other) {
    shape = gp_Other;
    loc.Multiply (T.matrix);
    loc.Add (T.loc);
    matrix.PreMultiply(T.matrix);
  }
  else {
    gp_Trsf2d T1 = Trsf2d();
    gp_Trsf2d T2 = T.Trsf2d();
    T1.PreMultiply(T2);
    matrix = T1.matrix;
    loc = T1.loc;
    scale = T1.scale;
    shape = T1.shape;
  }
}

gp_Trsf2d gp_GTrsf2d::Trsf2d () const
{
  // Test of orthogonality
  const double aTolerance  = Precision::Angular();
  const double aTolerance2 = 2.0 * aTolerance;

  if ( Form() == gp_Other )
    throw std::runtime_error("gp_GTrsf2d::Trsf2d() - non-orthogonal GTrsf2d(0)");

  double value = (matrix.Value(1, 1) * matrix.Value(1, 1)
                      +  matrix.Value(2, 1) * matrix.Value(2, 1));
  if ( Abs(value - 1.) > aTolerance2 )
    throw std::runtime_error("gp_GTrsf2d::Trsf2d() - non-orthogonal GTrsf2d(1)");

  value = (matrix.Value(1, 2) * matrix.Value(1, 2)
        +  matrix.Value(2, 2) * matrix.Value(2, 2));
  if ( Abs(value - 1.) > aTolerance2 )
    throw std::runtime_error("gp_GTrsf2d::Trsf2d() - non-orthogonal GTrsf2d(2)");

  value = (matrix.Value(1, 1) * matrix.Value(1, 2)
        +  matrix.Value(2, 1) * matrix.Value(2, 2));
  if ( Abs(value) > aTolerance )
    throw std::runtime_error("gp_GTrsf2d::Trsf2d() - non-orthogonal GTrsf2d(3)");

  gp_Trsf2d aTransformation;
  aTransformation.matrix = matrix;
  aTransformation.shape  = shape;
  aTransformation.scale  = scale;
  aTransformation.loc    = loc;

  return aTransformation;
}

