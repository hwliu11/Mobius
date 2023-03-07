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
// JCV 1/10/90 Changement de nom du package vgeom -> gp
// JCV 4/10/90 codage sur la forme de la transformation shape,Scaling,negative
// JCV 10/12/90 Modif introduction des classes Mat et XYZ dans gp

#define No_Standard_OutOfRange

#include <mobius/gp_Trsf.hxx>

#include <mobius/gp.hxx>
#include <mobius/gp_Ax1.hxx>
#include <mobius/gp_Ax2.hxx>
#include <mobius/gp_Ax3.hxx>
#include <mobius/gp_GTrsf.hxx>
#include <mobius/gp_Mat.hxx>
#include <mobius/gp_Pnt.hxx>
#include <mobius/gp_Quaternion.hxx>
#include <mobius/gp_Trsf2d.hxx>
#include <mobius/gp_Vec.hxx>
#include <mobius/gp_XYZ.hxx>

using namespace mobius::occ;

//=======================================================================
//function : gp_Trsf
// purpose :
//=======================================================================
gp_Trsf::gp_Trsf ()
: scale (1.0),
  shape (gp_Identity),
  matrix (1, 0, 0, 0, 1, 0, 0, 0, 1),
  loc (0.0, 0.0, 0.0)
{}

//=======================================================================
//function : SetMirror
// purpose :
//=======================================================================
void gp_Trsf::SetMirror (const gp_Pnt& theP)
{
  shape = gp_PntMirror;
  scale = -1.0;
  loc = theP.XYZ();
  matrix.SetIdentity();
  loc.Multiply (2.0);
}

//=======================================================================
//function : SetTranslation
// purpose :
//=======================================================================
void gp_Trsf::SetTranslation (const gp_Vec& theV) 
{
  shape = gp_Translation;
  scale = 1.;
  matrix.SetIdentity();
  loc = theV.XYZ();
}

//=======================================================================
//function : SetTranslation
// purpose :
//=======================================================================
void gp_Trsf::SetTranslation (const gp_Pnt& theP1,
                              const gp_Pnt& theP2) 
{
  shape = gp_Translation;
  scale = 1.0;
  matrix.SetIdentity();
  loc = (theP2.XYZ()).Subtracted (theP1.XYZ());
}

//=======================================================================
//function : Value
// purpose :
//=======================================================================
double gp_Trsf::Value (const int theRow, const int theCol) const
{
  if (theCol < 4)
  {
    return scale * matrix.Value (theRow, theCol);
  }
  else
  {
    return loc.Coord (theRow);
  }
}

//=======================================================================
//function : Transforms
// purpose :
//=======================================================================
void gp_Trsf::Transforms (double& theX,
                          double& theY,
                          double& theZ) const 
{
  gp_XYZ aTriplet (theX, theY, theZ);
  aTriplet.Multiply (matrix);
  if (scale != 1.0)
  {
    aTriplet.Multiply (scale);
  }
  aTriplet.Add (loc);
  theX = aTriplet.X();
  theY = aTriplet.Y();
  theZ = aTriplet.Z();
}

//=======================================================================
//function : Transforms
// purpose :
//=======================================================================
void gp_Trsf::Transforms (gp_XYZ& theCoord) const
{
  theCoord.Multiply (matrix);
  if (scale != 1.0)
  {
    theCoord.Multiply (scale);
  }
  theCoord.Add (loc);
}

//=======================================================================
//function : gp_Trsf
//purpose  : Constructor from 2d
//=======================================================================
gp_Trsf::gp_Trsf (const gp_Trsf2d& T) :
scale(T.ScaleFactor()),
shape(T.Form()),
loc(T.TranslationPart().X(),T.TranslationPart().Y(), 0.0)
{
  const gp_Mat2d& M = T.HVectorialPart();
  matrix(1,1) = M(1,1);
  matrix(1,2) = M(1,2);
  matrix(2,1) = M(2,1);
  matrix(2,2) = M(2,2);
  matrix(3,3) = 1.;
  if (shape == gp_Ax1Mirror)
  {
    scale = 1;
    matrix.Multiply(-1);
  }
}

//=======================================================================
//function : SetMirror
//purpose  : 
//=======================================================================

void gp_Trsf::SetMirror (const gp_Ax1& A1) 
{
  shape = gp_Ax1Mirror;
  scale = 1;
  loc = A1.Location().XYZ();
  matrix.SetDot(A1.Direction().XYZ());
  matrix.Multiply(-2);
  matrix.SetDiagonal (matrix.Value (1,1) + 1,
                      matrix.Value (2,2) + 1,
                      matrix.Value (3,3) + 1);

  loc.Multiply (matrix);
  loc.Add (A1.Location().XYZ());
  matrix.Multiply(-1);
}

//=======================================================================
//function : SetMirror
//purpose  : 
//=======================================================================

void gp_Trsf::SetMirror (const gp_Ax2& A2) 
{
  shape = gp_Ax2Mirror;
  scale = -1;
  loc = A2.Location().XYZ();
  matrix.SetDot(A2.Direction().XYZ());
  matrix.Multiply(2);
  matrix.SetDiagonal (matrix.Value (1,1) - 1,
                      matrix.Value (2,2) - 1,
                      matrix.Value (3,3) - 1);

  loc.Multiply (matrix);
  loc.Add (A2.Location().XYZ());
} 

//=======================================================================
//function : SetRotation
//purpose  : 
//=======================================================================

void gp_Trsf::SetRotation (const gp_Ax1& A1,
                           const double Ang)
{
  shape = gp_Rotation;
  scale = 1.;
  loc = A1.Location().XYZ();
  matrix.SetRotation (A1.Direction().XYZ(), Ang);
  loc.Reverse ();
  loc.Multiply (matrix);
  loc.Add (A1.Location().XYZ());
}

//=======================================================================
//function : SetRotation
//purpose  : 
//=======================================================================

void gp_Trsf::SetRotation (const gp_Quaternion& R)
{
  shape = gp_Rotation;
  scale = 1.;
  loc.SetCoord (0., 0., 0.);
  matrix = R.GetMatrix();
}

//=======================================================================
//function : SetRotationPart
//purpose  :
//=======================================================================
void gp_Trsf::SetRotationPart (const gp_Quaternion& theR)
{
  const bool hasRotation = !theR.IsEqual (gp_Quaternion());
  if (hasRotation)
  {
    matrix = theR.GetMatrix();
  }
  else
  {
    matrix.SetIdentity();
  }

  switch (shape)
  {
    case gp_Identity:
    {
      if (hasRotation)
      {
        shape = gp_Rotation;
      }
      break;
    }
    case gp_Rotation:
    {
      if (!hasRotation)
      {
        shape = gp_Identity;
      }
      break;
    }
    case gp_Translation:
    case gp_PntMirror:
    case gp_Ax1Mirror:
    case gp_Ax2Mirror:
    case gp_Scale:
    case gp_CompoundTrsf:
    case gp_Other:
    {
      if (hasRotation)
      {
        shape = gp_CompoundTrsf;
      }
      break;
    }
  }
}

//=======================================================================
//function : SetScale
//purpose  : 
//=======================================================================

void gp_Trsf::SetScale (const gp_Pnt& P, const double S)  
{
  shape = gp_Scale;
  scale = S;
  loc = P.XYZ();
  double As = scale;
  if (As < 0) As = - As;
  matrix.SetIdentity ();
  loc.Multiply (1-S);
}

//=======================================================================
//function : SetTransformation
//purpose  : 
//=======================================================================

void gp_Trsf::SetTransformation (const gp_Ax3& FromA1,
                                 const gp_Ax3& ToA2)
{
  shape = gp_CompoundTrsf;
  scale = 1.0;
  // matrix from XOY  ToA2 :
  matrix.SetRows (ToA2.XDirection().XYZ(),
                  ToA2.YDirection().XYZ(),
                  ToA2. Direction().XYZ());
  loc = ToA2.Location().XYZ();
  loc.Multiply (matrix);
  loc.Reverse ();

  // matrix FromA1 to XOY :
  const gp_XYZ& xDir = FromA1.XDirection().XYZ();
  const gp_XYZ& yDir = FromA1.YDirection().XYZ();
  const gp_XYZ& zDir = FromA1.Direction().XYZ();

  gp_Mat MA1 (xDir, yDir, zDir);
  gp_XYZ MA1loc = FromA1.Location().XYZ();

  // matrix * MA1 => FromA1 ToA2 :
  MA1loc.Multiply (matrix);
  loc.Add (MA1loc);
  matrix.Multiply (MA1);
}

void gp_Trsf::SetTransformation (const gp_Ax3& A3) 
{
  shape = gp_CompoundTrsf;
  scale = 1.0;
  matrix.SetRows (A3.XDirection().XYZ(),
                  A3.YDirection().XYZ(),
                  A3. Direction().XYZ());
  loc = A3.Location().XYZ();
  loc.Multiply (matrix);
  loc.Reverse ();
}

//=======================================================================
//function : SetTransformation
//purpose  : 
//=======================================================================

void gp_Trsf::SetTransformation (const gp_Quaternion& R, const gp_Vec& T)
{
  shape = gp_CompoundTrsf;
  scale = 1.;
  loc = T.XYZ();
  matrix = R.GetMatrix();
}

//=======================================================================
//function : SetDisplacement
//purpose  : 
//=======================================================================

void gp_Trsf::SetDisplacement (const gp_Ax3& FromA1,
                               const gp_Ax3& ToA2)
{
  shape = gp_CompoundTrsf;
  scale = 1.0;
  // matrix from ToA2 to XOY :
  matrix.SetCol (1, ToA2.XDirection().XYZ());
  matrix.SetCol (2, ToA2.YDirection().XYZ());
  matrix.SetCol (3, ToA2.Direction().XYZ());
  loc = ToA2.Location().XYZ();
  // matrix XOY to FromA1 :
  const gp_XYZ& xDir = FromA1.XDirection().XYZ();
  const gp_XYZ& yDir = FromA1.YDirection().XYZ();
  const gp_XYZ& zDir = FromA1.Direction().XYZ();
  gp_Mat MA1 (xDir, yDir, zDir);
  MA1.Transpose();
  gp_XYZ MA1loc = FromA1.Location().XYZ();
  MA1loc.Multiply (MA1);
  MA1loc.Reverse();
  // matrix * MA1 
  MA1loc.Multiply (matrix);
  loc.Add (MA1loc);
  matrix.Multiply (MA1);
}

//=======================================================================
//function : SetTranslationPart
//purpose  : 
//=======================================================================

void gp_Trsf::SetTranslationPart (const gp_Vec& V) {   

  loc = V.XYZ();
  const bool locnull = (loc.SquareModulus() < gp::Resolution());

  switch (shape) {

  case gp_Identity :
    if (!locnull) shape = gp_Translation;
    break;

  case gp_Translation :
    if (locnull) shape = gp_Identity;
    break;

  case gp_Rotation :
  case gp_PntMirror :
  case gp_Ax1Mirror :
  case gp_Ax2Mirror :
  case gp_Scale :
  case gp_CompoundTrsf :
  case gp_Other :
    if (!locnull) {
      shape = gp_CompoundTrsf;
    }
    break;
  }
}

//=======================================================================
//function : SetScaleFactor
//purpose  : 
//=======================================================================

void gp_Trsf::SetScaleFactor (const double S) 
{   
  double As = S;
  if (As < 0) As = - As;
  scale = S;
  As = scale - 1.;
  if (As < 0) As = - As;
  bool unit  = As <= gp::Resolution(); // = (scale == 1)
  As = scale + 1.;
  if (As < 0) As = - As;
  bool munit = As <= gp::Resolution(); // = (scale == -1)
  
  switch (shape) {
  case gp_Identity :
  case gp_Translation :
    if (!unit) shape = gp_Scale;
    if (munit) shape = gp_PntMirror;
    break;
  case gp_Rotation :
    if (!unit) shape = gp_CompoundTrsf;
    break;
  case gp_PntMirror :
  case gp_Ax1Mirror :
  case gp_Ax2Mirror :
    if (!munit) shape = gp_Scale;
    if (unit)   shape = gp_Identity;
    break;
  case gp_Scale :
    if (unit)  shape = gp_Identity;
    if (munit) shape = gp_PntMirror;
    break;
  case gp_CompoundTrsf :
    break;
  case gp_Other :
    break;
  }
}

//=======================================================================
//function : SetValues
//purpose  : 
// 06-01-1998 modified by PMN : On utilise TolDist pour evaluer si les coeffs 
//  sont nuls : c'est toujours mieux que gp::Resolution !
//=======================================================================

void gp_Trsf::SetValues(const double a11, 
                        const double a12, 
                        const double a13, 
                        const double a14, 
                        const double a21, 
                        const double a22, 
                        const double a23, 
                        const double a24, 
                        const double a31, 
                        const double a32,
                        const double a33, 
                        const double a34)
{
  gp_XYZ col1(a11,a21,a31);
  gp_XYZ col2(a12,a22,a32);
  gp_XYZ col3(a13,a23,a33);
  gp_XYZ col4(a14,a24,a34);
  // compute the determinant
  gp_Mat M(col1,col2,col3);
  double s = M.Determinant();
  double As = s;
  if (As < 0) As = - As;
  if (s > 0)
    s = Pow(s,1./3.);
  else
    s = -Pow(-s,1./3.);
  M.Divide(s);
  
  scale = s;
  shape = gp_CompoundTrsf;

  matrix = M;
  Orthogonalize();
  
  loc = col4;
}

//=======================================================================
//function : GetRotation
//purpose  : 
//=======================================================================

gp_Quaternion gp_Trsf::GetRotation () const
{ 
  return gp_Quaternion (matrix); 
}

//=======================================================================
//function : VectorialPart
//purpose  : 
//=======================================================================

gp_Mat gp_Trsf::VectorialPart () const
{ 
  if (scale == 1.0)  return matrix; 
  gp_Mat M = matrix;
  if (shape == gp_Scale || shape == gp_PntMirror)
    M.SetDiagonal(scale*M.Value(1,1),
                  scale*M.Value(2,2),
                  scale*M.Value(3,3));
  else
    M.Multiply (scale);
  return M;
}

//=======================================================================
//function : Invert
//purpose  : 
//=======================================================================

void gp_Trsf::Invert()
{ 
  //                                    -1
  //  X' = scale * R * X + T  =>  X = (R  / scale)  * ( X' - T)
  //
  // Pour les gp_Trsf puisque le scale est extrait de la gp_Matrice R
  // on a toujours determinant (R) = 1 et R-1 = R transposee.
  if (shape == gp_Identity) { }
  else if (shape == gp_Translation || shape == gp_PntMirror) loc.Reverse();
  else if (shape == gp_Scale) {
    scale = 1.0 / scale;
    loc.Multiply (-scale);
  }
  else {
    scale = 1.0 / scale;
    matrix.Transpose ();
    loc.Multiply (matrix);
    loc.Multiply (-scale);
  }
}

//=======================================================================
//function : Multiply
//purpose  : 
//=======================================================================

void gp_Trsf::Multiply(const gp_Trsf& T)
{
  if (T.shape == gp_Identity) { }
  else if (shape == gp_Identity) {
    shape = T.shape;
    scale = T.scale;
    loc = T.loc;
    matrix = T.matrix;
  } 
  else if (shape == gp_Rotation && T.shape == gp_Rotation) { 
    if (T.loc.X() != 0.0 || T.loc.Y() != 0.0 || T.loc.Z() != 0.0) {
      loc.Add (T.loc.Multiplied (matrix));
    }
    matrix.Multiply(T.matrix);
  }
  else if (shape == gp_Translation && T.shape == gp_Translation) {
    loc.Add (T.loc);
  }
  else if (shape == gp_Scale && T.shape == gp_Scale) {
    loc.Add (T.loc.Multiplied(scale));
    scale = scale * T.scale;
  }
  else if (shape == gp_PntMirror && T.shape == gp_PntMirror) {
    scale = 1.0;
    shape = gp_Translation;
    loc.Add (T.loc.Reversed());
  }
  else if (shape == gp_Ax1Mirror && T.shape == gp_Ax1Mirror) {
    shape = gp_Rotation;
    loc.Add (T.loc.Multiplied (matrix));
    matrix.Multiply(T.matrix);
  }
  else if ((shape == gp_CompoundTrsf || shape == gp_Rotation ||
            shape == gp_Ax1Mirror || shape == gp_Ax2Mirror)
           && T.shape == gp_Translation) {
    gp_XYZ Tloc(T.loc);
    Tloc.Multiply(matrix);
    if (scale != 1.0) { Tloc.Multiply(scale); }
    loc.Add (Tloc);
  }
  else if ((shape == gp_Scale || shape == gp_PntMirror)
           && T.shape == gp_Translation) {
    gp_XYZ Tloc(T.loc);
    Tloc.Multiply (scale);
    loc.Add (Tloc);
  }
  else if (shape == gp_Translation && 
           (T.shape == gp_CompoundTrsf || T.shape == gp_Rotation ||
            T.shape == gp_Ax1Mirror || T.shape == gp_Ax2Mirror)) {
    shape = gp_CompoundTrsf;
    scale = T.scale;
    loc.Add (T.loc);
    matrix = T.matrix;
  }
  else if (shape == gp_Translation && 
           (T.shape == gp_Scale || T.shape == gp_PntMirror)) {
    shape = T.shape;
    loc.Add (T.loc);
    scale = T.scale;
  }
  else if ((shape == gp_PntMirror || shape == gp_Scale) &&
           (T.shape == gp_PntMirror || T.shape == gp_Scale)) {
    shape = gp_CompoundTrsf;
    gp_XYZ Tloc(T.loc);
    Tloc.Multiply (scale);
    loc.Add (Tloc);
    scale = scale * T.scale;
  }
  else if ((shape == gp_CompoundTrsf || shape == gp_Rotation ||
            shape == gp_Ax1Mirror || shape == gp_Ax2Mirror)
           && (T.shape == gp_Scale || T.shape == gp_PntMirror)) {
    shape = gp_CompoundTrsf;
    gp_XYZ Tloc(T.loc);
    if (scale == 1.0) {
      scale = T.scale;
      Tloc.Multiply(matrix);
    }
    else {
      Tloc.Multiply (matrix);
      Tloc.Multiply (scale);
      scale = scale * T.scale;
    }
    loc.Add (Tloc);
  }
  else if ((T.shape == gp_CompoundTrsf || T.shape == gp_Rotation ||
            T.shape == gp_Ax1Mirror || T.shape == gp_Ax2Mirror)
           && (shape == gp_Scale || shape == gp_PntMirror)) {
    shape = gp_CompoundTrsf;
    gp_XYZ Tloc(T.loc);
    Tloc.Multiply(scale);
    loc.Add (Tloc);
    scale = scale * T.scale;
    matrix = T.matrix;
  }
  else {
    shape = gp_CompoundTrsf;
    gp_XYZ Tloc(T.loc);
    Tloc.Multiply (matrix);
    if (scale != 1.0) { 
      Tloc.Multiply (scale);
      scale = scale * T.scale;
    }
    else { scale = T.scale; }
    loc.Add (Tloc);
    matrix.Multiply(T.matrix);
  }
}

//=======================================================================
//function : Power
//purpose  : 
//=======================================================================

void gp_Trsf::Power (const int N)
{
  if (shape == gp_Identity) { }
  else {
    if (N == 0)  {
      scale = 1.0;
      shape = gp_Identity;
      matrix.SetIdentity();
      loc = gp_XYZ (0.0, 0.0, 0.0);
    }
    else if (N == 1)  { }
    else if (N == -1) { Invert(); }
    else {
      if (N < 0) { Invert(); }
      if (shape == gp_Translation) {
        int Npower = N;
        if (Npower < 0) Npower = - Npower;
        Npower--;
        gp_XYZ Temploc = loc;
        for(;;) {
          if (IsOdd(Npower))  loc.Add (Temploc);
          if (Npower == 1) break;
          Temploc.Add (Temploc);
          Npower = Npower/2;
        }
      }
      else if (shape == gp_Scale) {
        int Npower = N;
        if (Npower < 0) Npower = - Npower;
        Npower--;
        gp_XYZ Temploc = loc;
        double Tempscale = scale;
        for(;;) {
          if (IsOdd(Npower)) {
            loc.Add (Temploc.Multiplied(scale));
            scale = scale * Tempscale;
          }
          if (Npower == 1) break;
          Temploc.Add (Temploc.Multiplied(Tempscale));
          Tempscale = Tempscale * Tempscale;
          Npower = Npower/2;
        }
      }
      else if (shape == gp_Rotation) {
        int Npower = N;
        if (Npower < 0) Npower = - Npower;
        Npower--;
        gp_Mat Tempmatrix (matrix);
        if (loc.X() == 0.0 && loc.Y() == 0.0 && loc.Z() == 0.0) {
          for(;;) {
            if (IsOdd(Npower)) matrix.Multiply (Tempmatrix);
            if (Npower == 1)   break;
            Tempmatrix.Multiply (Tempmatrix);
            Npower = Npower/2;
          }
        }
        else {
          gp_XYZ Temploc = loc;
          for(;;) {
            if (IsOdd(Npower)) {
              loc.Add (Temploc.Multiplied (matrix));
              matrix.Multiply (Tempmatrix);
            }
            if (Npower == 1) break;
            Temploc.Add (Temploc.Multiplied (Tempmatrix));
            Tempmatrix.Multiply (Tempmatrix);
            Npower = Npower/2;
          }
        }
      }
      else if (shape == gp_PntMirror || shape == gp_Ax1Mirror ||
               shape == gp_Ax2Mirror) {
        if (IsEven (N)) {
          shape = gp_Identity;
          scale = 1.0;
          matrix.SetIdentity ();
          loc.SetX(0);
          loc.SetY(0);
          loc.SetZ(0);
        }
      }
      else {
        shape = gp_CompoundTrsf;
        int Npower = N;
        if (Npower < 0) Npower = - Npower;
        Npower--;
        gp_XYZ Temploc = loc;
        double Tempscale = scale;
        gp_Mat Tempmatrix (matrix);
        for(;;) {
          if (IsOdd(Npower)) {
            loc.Add ((Temploc.Multiplied (matrix)).Multiplied (scale));
            scale = scale * Tempscale;
            matrix.Multiply (Tempmatrix);
          }
          if (Npower == 1) break;
          Tempscale = Tempscale * Tempscale;
          Temploc.Add ( (Temploc.Multiplied (Tempmatrix)).Multiplied 
                        (Tempscale)
                        );
          Tempmatrix.Multiply (Tempmatrix);
          Npower = Npower/2;
        }
      }
    }
  }
}

//=======================================================================
//function : PreMultiply
//purpose  : 
//=======================================================================

void gp_Trsf::PreMultiply (const gp_Trsf& T)
{
  if (T.shape == gp_Identity) { }
  else if (shape == gp_Identity) {
    shape = T.shape;
    scale = T.scale;
    loc = T.loc;
    matrix = T.matrix;
  } 
  else if (shape == gp_Rotation && T.shape == gp_Rotation) { 
    loc.Multiply (T.matrix);
    loc.Add (T.loc);
    matrix.PreMultiply(T.matrix);
  }
  else if (shape == gp_Translation && T.shape == gp_Translation) {
    loc.Add (T.loc);
  }
  else if (shape == gp_Scale && T.shape == gp_Scale) {
    loc.Multiply (T.scale);
    loc.Add (T.loc);
    scale = scale * T.scale;
  }
  else if (shape == gp_PntMirror && T.shape == gp_PntMirror) {
    scale = 1.0;
    shape = gp_Translation;
    loc.Reverse();
    loc.Add (T.loc);
  }
  else if (shape == gp_Ax1Mirror && T.shape == gp_Ax1Mirror) {
    shape = gp_Rotation;
    loc.Multiply (T.matrix);
    loc.Add (T.loc);
    matrix.PreMultiply(T.matrix);
  }
  else if ((shape == gp_CompoundTrsf || shape == gp_Rotation ||
            shape == gp_Ax1Mirror || shape == gp_Ax2Mirror)
           && T.shape == gp_Translation) {
    loc.Add (T.loc);
  }
  else if ((shape == gp_Scale || shape == gp_PntMirror)
           && T.shape == gp_Translation) {
    loc.Add (T.loc);
  }
  else if (shape == gp_Translation && 
           (T.shape == gp_CompoundTrsf || T.shape == gp_Rotation
            || T.shape == gp_Ax1Mirror || T.shape == gp_Ax2Mirror)) {
    shape = gp_CompoundTrsf;
    matrix = T.matrix;
    if (T.scale == 1.0)  loc.Multiply (T.matrix);
    else {
      scale = T.scale;
      loc.Multiply (matrix);
      loc.Multiply (scale);
    }
    loc.Add (T.loc);
  }
  else if ((T.shape == gp_Scale || T.shape == gp_PntMirror)
           && shape == gp_Translation) {
    loc.Multiply (T.scale);
    loc.Add (T.loc);
    scale = T.scale;
    shape = T.shape;
  }
  else if ((shape == gp_PntMirror || shape == gp_Scale) &&
           (T.shape == gp_PntMirror || T.shape == gp_Scale)) {
    shape = gp_CompoundTrsf;
    loc.Multiply (T.scale);
    loc.Add (T.loc);
    scale = scale * T.scale;
  }
  else if ((shape == gp_CompoundTrsf || shape == gp_Rotation ||
            shape == gp_Ax1Mirror || shape == gp_Ax2Mirror) 
           && (T.shape == gp_Scale || T.shape == gp_PntMirror)) {
    shape = gp_CompoundTrsf;
    loc.Multiply (T.scale);
    loc.Add (T.loc);
    scale = scale * T.scale;
  } 
  else if ((T.shape == gp_CompoundTrsf || T.shape == gp_Rotation ||
            T.shape == gp_Ax1Mirror || T.shape == gp_Ax2Mirror) 
           && (shape == gp_Scale || shape == gp_PntMirror)) {
    shape = gp_CompoundTrsf;
    matrix = T.matrix;
    if (T.scale == 1.0)  loc.Multiply (T.matrix);
    else {
      loc.Multiply (matrix);
      loc.Multiply (T.scale);
      scale = T.scale * scale;
    }
    loc.Add (T.loc);
  } 
  else {
    shape = gp_CompoundTrsf;
    loc.Multiply (T.matrix);
    if (T.scale != 1.0) {
      loc.Multiply (T.scale);    scale = scale * T.scale;
    }
    loc.Add (T.loc);
    matrix.PreMultiply(T.matrix);
  }
}

//=======================================================================
//function : GetRotation
//purpose  : algorithm from A.Korn, M.Korn, "Mathematical Handbook for
//           scientists and Engineers" McGraw-Hill, 1961, ch.14.10-2.
//=======================================================================

bool gp_Trsf::GetRotation (gp_XYZ&        theAxis,
                                       double& theAngle) const
{
  gp_Quaternion Q = GetRotation();
  gp_Vec aVec;
  Q.GetVectorAndAngle (aVec, theAngle);
  theAxis = aVec.XYZ();
  return true;
}

//=======================================================================
//function : Orthogonalize
//purpose  : 
//ATTENTION!!!
//      Orthogonalization is not equivalent transformation. Therefore, 
//        transformation with source matrix and with orthogonalized matrix can
//        lead to different results for one shape. Consequently, source matrix must
//        be close to orthogonalized matrix for reducing these differences.
//=======================================================================
void gp_Trsf::Orthogonalize()
{
  //Matrix M is called orthogonal if and only if
  //    M*Transpose(M) == E
  //where E is identity matrix.

  //Set of all rows (as of all columns) of matrix M (for gp_Trsf class) is
  //orthonormal basis. If this condition is not satisfied then the basis can be
  //orthonormalized in accordance with below described algorithm.

  //In 3D-space, we have the linear span of three basis vectors: V1, V2 and V3.
  //Correspond orthonormalized basis is formed by vectors Vn1, Vn2 and Vn3.

  //In this case,
  //    Vn_{i}*Vn_{j} = (i == j)? 1 : 0.

  //The algorithm includes following steps:

  //1. Normalize V1 vector:
  //    V1n=V1/|V1|;
  //
  //2. Let
  //    V2n=V2-m*V1n.
  //    
  //After multiplication two parts of this equation by V1n,
  //we will have following equation:
  //    0=V2*V1n-m <==> m=V2*V1n.
  //    
  //Consequently,
  //    V2n=V2-(V2*V1n)*V1n.

  //3. Let
  //    V3n=V3-m1*V1n-m2*V2n.
  //    
  //After multiplication two parts of this equation by V1n,
  //we will have following equation:
  //    0=V3*V1n-m1 <==> m1=V3*V1n.
  //    
  //After multiplication two parts of main equation by V2n,
  //we will have following equation:
  //    0=V3*V2n-m2 <==> m2=V3*V2n.
  //    
  //In conclusion,
  //    V3n=V3-(V3*V1n)*V1n-(V3*V2n)*V2n.

  gp_Mat aTM(matrix);

  gp_XYZ aV1 = aTM.Column(1);
  gp_XYZ aV2 = aTM.Column(2);
  gp_XYZ aV3 = aTM.Column(3);

  aV1.Normalize();

  aV2 -= aV1*(aV2.Dot(aV1));
  aV2.Normalize();

  aV3 -= aV1*(aV3.Dot(aV1)) + aV2*(aV3.Dot(aV2));
  aV3.Normalize();

  aTM.SetCols(aV1, aV2, aV3);

  aV1 = aTM.Row(1);
  aV2 = aTM.Row(2);
  aV3 = aTM.Row(3);

  aV1.Normalize();

  aV2 -= aV1*(aV2.Dot(aV1));
  aV2.Normalize();

  aV3 -= aV1*(aV3.Dot(aV1)) + aV2*(aV3.Dot(aV2));
  aV3.Normalize();

  aTM.SetRows(aV1, aV2, aV3);

  matrix = aTM;
}
