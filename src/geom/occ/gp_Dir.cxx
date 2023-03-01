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

// JCV 07/12/90 Modifs suite a l'introduction des classes XYZ et Mat dans gp

#include <mobius/gp_Dir.hxx>

#include <mobius/gp_Ax1.hxx>
#include <mobius/gp_Ax2.hxx>
#include <mobius/gp_Trsf.hxx>
#include <mobius/gp_Vec.hxx>
#include <mobius/gp_XYZ.hxx>

using namespace mobius::occ;

// =======================================================================
// function : gp_Dir
// purpose  :
// =======================================================================
gp_Dir::gp_Dir(const gp_Vec& theV)
{
  const gp_XYZ& aXYZ = theV.XYZ();
  double aX = aXYZ.X();
  double aY = aXYZ.Y();
  double aZ = aXYZ.Z();
  double aD = sqrt (aX * aX + aY * aY + aZ * aZ);
  coord.SetX (aX / aD);
  coord.SetY (aY / aD);
  coord.SetZ (aZ / aD);
}

// =======================================================================
// function : gp_Dir
// purpose  :
// =======================================================================
gp_Dir::gp_Dir (const gp_XYZ& theXYZ)
{
  double aX = theXYZ.X();
  double aY = theXYZ.Y();
  double aZ = theXYZ.Z();
  double aD = sqrt (aX * aX + aY * aY + aZ * aZ);
  coord.SetX (aX / aD);
  coord.SetY (aY / aD);
  coord.SetZ (aZ / aD);
}

// =======================================================================
// function : gp_Dir
// purpose  :
// =======================================================================
gp_Dir::gp_Dir (const double theXv,
                       const double theYv,
                       const double theZv)
{
  double aD = sqrt (theXv * theXv + theYv * theYv + theZv * theZv);
  coord.SetX (theXv / aD);
  coord.SetY (theYv / aD);
  coord.SetZ (theZv / aD);
}

// =======================================================================
// function : SetCoord
// purpose  :
// =======================================================================
void gp_Dir::SetCoord (const int theIndex,
                              const double theXi)
{
  double aX = coord.X();
  double aY = coord.Y();
  double aZ = coord.Z();
  if (theIndex == 1)
  {
    aX = theXi;
  }
  else if (theIndex == 2)
  {
    aY = theXi;
  }
  else
  {
    aZ = theXi;
  }
  double aD = sqrt (aX * aX + aY * aY + aZ * aZ);
  coord.SetX (aX / aD);
  coord.SetY (aY / aD);
  coord.SetZ (aZ / aD);
}

// =======================================================================
// function : SetCoord
// purpose  :
// =======================================================================
void gp_Dir::SetCoord (const double theXv,
                              const double theYv,
                              const double theZv) {
  double aD = sqrt (theXv * theXv + theYv * theYv + theZv * theZv);
  coord.SetX (theXv / aD);
  coord.SetY (theYv / aD);
  coord.SetZ (theZv / aD);
}

// =======================================================================
// function : SetX
// purpose  :
// =======================================================================
void gp_Dir::SetX (const double theX)
{
  double anY = coord.Y();
  double aZ = coord.Z();
  double aD = sqrt (theX * theX + anY * anY + aZ * aZ);
  coord.SetX (theX / aD);
  coord.SetY (anY / aD);
  coord.SetZ (aZ / aD);
}

// =======================================================================
// function : SetY
// purpose  :
// =======================================================================
void gp_Dir::SetY (const double theY)
{
  double aZ = coord.Z();
  double aX = coord.X();
  double aD = sqrt (aX * aX + theY * theY + aZ * aZ);
  coord.SetX (aX / aD);
  coord.SetY (theY / aD);
  coord.SetZ (aZ / aD);
}

// =======================================================================
// function : SetZ
// purpose  :
// =======================================================================
void gp_Dir::SetZ (const double theZ)
{
  double aX = coord.X();
  double anY = coord.Y();
  double aD = sqrt (aX * aX + anY * anY + theZ * theZ);
  coord.SetX (aX / aD);
  coord.SetY (anY / aD);
  coord.SetZ (theZ / aD);
}

// =======================================================================
// function : SetXYZ
// purpose  :
// =======================================================================
void gp_Dir::SetXYZ (const gp_XYZ& theXYZ)
{
  double aX = theXYZ.X();
  double anY = theXYZ.Y();
  double aZ = theXYZ.Z();
  double aD = sqrt(aX * aX + anY * anY + aZ * aZ);
  coord.SetX (aX / aD);
  coord.SetY (anY / aD);
  coord.SetZ (aZ / aD);
}

// =======================================================================
// function : Cross
// purpose  :
// =======================================================================
void gp_Dir::Cross(const gp_Dir& theRight)
{
  coord.Cross (theRight.coord);
  double aD = coord.Modulus();
  coord.Divide (aD);
}

// =======================================================================
// function : Crossed
// purpose  :
// =======================================================================
gp_Dir gp_Dir::Crossed (const gp_Dir& theRight) const
{
  gp_Dir aV = *this;
  aV.coord.Cross (theRight.coord);
  double aD = aV.coord.Modulus();
  aV.coord.Divide (aD);
  return aV;
}

// =======================================================================
// function : CrossCross
// purpose  :
// =======================================================================
void gp_Dir::CrossCross (const gp_Dir& theV1, const gp_Dir& theV2)
{
  coord.CrossCross (theV1.coord, theV2.coord);
  double aD = coord.Modulus();
  coord.Divide (aD);
}

// =======================================================================
// function : CrossCrossed
// purpose  :
// =======================================================================
gp_Dir gp_Dir::CrossCrossed (const gp_Dir& theV1, const gp_Dir& theV2) const
{
  gp_Dir aV = *this;
  (aV.coord).CrossCross (theV1.coord, theV2.coord);
  double aD = aV.coord.Modulus();
  aV.coord.Divide (aD);
  return aV;
}

// =======================================================================
// function : Rotate
// purpose  :
// =======================================================================
void gp_Dir::Rotate(const gp_Ax1& theA1, const double theAng)
{
  gp_Trsf aT;
  aT.SetRotation (theA1, theAng);
  coord.Multiply (aT.HVectorialPart());
}

double gp_Dir::Angle (const gp_Dir& Other) const
{
  //    Commentaires :
  //    Au dessus de 45 degres l'arccos donne la meilleur precision pour le
  //    calcul de l'angle. Sinon il vaut mieux utiliser l'arcsin.
  //    Les erreurs commises sont loin d'etre negligeables lorsque l'on est
  //    proche de zero ou de 90 degres.
  //    En 3d les valeurs angulaires sont toujours positives et comprises entre
  //    0 et PI
  double Cosinus = coord.Dot (Other.coord);
  if (Cosinus > -0.70710678118655 && Cosinus < 0.70710678118655)
    return acos (Cosinus);
  else {
    double Sinus = (coord.Crossed (Other.coord)).Modulus ();
    if(Cosinus < 0.0)  return M_PI - asin (Sinus);
    else               return      asin (Sinus);
  }
}

double gp_Dir::AngleWithRef (const gp_Dir& Other,
				    const gp_Dir& Vref) const
{
  double Ang;
  gp_XYZ XYZ = coord.Crossed (Other.coord);
  double Cosinus = coord.Dot(Other.coord);
  double Sinus   = XYZ.Modulus ();
  if (Cosinus > -0.70710678118655 && Cosinus < 0.70710678118655)
    Ang =  acos (Cosinus);
  else {
    if(Cosinus < 0.0)  Ang = M_PI - asin (Sinus);
    else               Ang =      asin (Sinus);
  }
  if (XYZ.Dot (Vref.coord) >= 0.0)  return  Ang;
  else                              return -Ang;
} 

void gp_Dir::Mirror (const gp_Dir& V)
{
  const gp_XYZ& XYZ = V.coord;
  double A = XYZ.X();
  double B = XYZ.Y();
  double C = XYZ.Z();
  double X = coord.X();
  double Y = coord.Y();
  double Z = coord.Z();
  double M1 = 2.0 * A * B;
  double M2 = 2.0 * A * C;
  double M3 = 2.0 * B * C;
  double XX = ((2.0 * A * A) - 1.0) * X + M1 * Y + M2 * Z;
  double YY = M1 * X + ((2.0 * B * B) - 1.0) * Y + M3 * Z;
  double ZZ = M2 * X + M3 * Y + ((2.0 * C * C) - 1.0) * Z;
  coord.SetCoord(XX,YY,ZZ);
}

void gp_Dir::Mirror (const gp_Ax1& A1)
{
  const gp_XYZ& XYZ = A1.Direction().coord;
  double A = XYZ.X();
  double B = XYZ.Y();
  double C = XYZ.Y();
  double X = coord.X();
  double Y = coord.Y();
  double Z = coord.Z();
  double M1 = 2.0 * A * B;
  double M2 = 2.0 * A * C;
  double M3 = 2.0 * B * C;
  double XX = ((2.0 * A * A) - 1.0) * X + M1 * Y + M2 * Z;
  double YY = M1 * X + ((2.0 * B * B) - 1.0) * Y + M3 * Z;
  double ZZ = M2 * X + M3 * Y + ((2.0 * C * C) - 1.0) * Z;
  coord.SetCoord(XX,YY,ZZ);
}

void gp_Dir::Mirror (const gp_Ax2& A2)
{
  const gp_Dir& Vz = A2.Direction();
  Mirror(Vz);
  Reverse();
}

void gp_Dir::Transform (const gp_Trsf& T)
{
  if (T.Form() == gp_Identity ||  T.Form() == gp_Translation)    { }
  else if (T.Form() == gp_PntMirror) { coord.Reverse(); }
  else if (T.Form() == gp_Scale) { 
    if (T.ScaleFactor() < 0.0) { coord.Reverse(); }
  }
  else {
    coord.Multiply (T.HVectorialPart());
    double D = coord.Modulus();
    coord.Divide(D);
    if (T.ScaleFactor() < 0.0) { coord.Reverse(); }
  } 
}

gp_Dir gp_Dir::Mirrored (const gp_Dir& V) const
{
  gp_Dir Vres = *this;
  Vres.Mirror (V);
  return Vres;
}

gp_Dir gp_Dir::Mirrored (const gp_Ax1& A1) const
{
  gp_Dir V = *this;
  V.Mirror (A1);
  return V;
}

gp_Dir gp_Dir::Mirrored (const gp_Ax2& A2) const
{
  gp_Dir V = *this;
  V.Mirror (A2);
  return V;
}
