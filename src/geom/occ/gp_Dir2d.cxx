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

// JCV 08/01/90 Modifs suite a l'introduction des classes XY et Mat2d dans gp

#include <mobius/gp_Dir2d.hxx>

#include <mobius/gp_Ax2d.hxx>
#include <mobius/gp_Trsf2d.hxx>
#include <mobius/gp_Vec2d.hxx>
#include <mobius/gp_XY.hxx>

using namespace mobius::occ;

// =======================================================================
// function : gp_Dir2d
// purpose  :
// =======================================================================
inline gp_Dir2d::gp_Dir2d (const gp_Vec2d& theV)
{
  const gp_XY& aXY = theV.XY();
  double aX = aXY.X();
  double anY = aXY.Y();
  double aD = sqrt (aX * aX + anY * anY);
  coord.SetX (aX / aD);
  coord.SetY (anY / aD);
}

// =======================================================================
// function : gp_Dir2d
// purpose  :
// =======================================================================
inline gp_Dir2d::gp_Dir2d (const gp_XY& theXY)
{
  double aX = theXY.X();
  double anY = theXY.Y();
  double aD = sqrt (aX * aX + anY * anY);
  coord.SetX (aX / aD);
  coord.SetY (anY / aD);
}

// =======================================================================
// function : gp_Dir2d
// purpose  :
// =======================================================================
inline gp_Dir2d::gp_Dir2d (const double theXv,
                           const double theYv)
{
  double aD = sqrt (theXv * theXv + theYv * theYv);
  coord.SetX (theXv / aD);
  coord.SetY (theYv / aD);
}

// =======================================================================
// function : SetCoord
// purpose  :
// =======================================================================
inline void gp_Dir2d::SetCoord (const int theIndex,
                                const double theXi)
{
  double aX = coord.X();
  double anY = coord.Y();
  if (theIndex == 1)
  {
    aX = theXi;
  }
  else
  {
    anY = theXi;
  }
  double aD = sqrt (aX * aX + anY * anY);
  coord.SetX (aX / aD);
  coord.SetY (anY / aD);
}

// =======================================================================
// function : SetCoord
// purpose  :
// =======================================================================
inline void gp_Dir2d::SetCoord (const double theXv,
                                const double theYv)
{
  double aD = sqrt (theXv * theXv + theYv * theYv);
  coord.SetX (theXv / aD);
  coord.SetY (theYv / aD);
}

// =======================================================================
// function : SetX
// purpose  :
// =======================================================================
inline void gp_Dir2d::SetX (const double theX)
{
  double anY = coord.Y();
  double aD = sqrt (theX * theX + anY * anY);
  coord.SetX (theX / aD);
  coord.SetY (anY / aD);
}

// =======================================================================
// function : SetY
// purpose  :
// =======================================================================
inline void gp_Dir2d::SetY (const double theY)
{
  double aX = coord.X();
  double aD = sqrt (aX * aX + theY * theY);
  coord.SetX (aX / aD);
  coord.SetY (theY / aD);
}

// =======================================================================
// function : SetXY
// purpose  :
// =======================================================================
inline void gp_Dir2d::SetXY (const gp_XY& theXY)
{
  double aX = theXY.X();
  double anY = theXY.Y();
  double aD = sqrt (aX * aX + anY * anY);
  coord.SetX (aX / aD);
  coord.SetY (anY / aD);
}

// =======================================================================
// function : IsEqual
// purpose  :
// =======================================================================
inline bool gp_Dir2d::IsEqual (const gp_Dir2d& theOther,
                                           const double theAngularTolerance) const
{
  double anAng = Angle (theOther);
  if (anAng < 0)
  {
    anAng = -anAng;
  }
  return anAng <= theAngularTolerance;
}

// =======================================================================
// function : IsNormal
// purpose  :
// =======================================================================
inline bool gp_Dir2d::IsNormal (const gp_Dir2d& theOther,
                                            const double theAngularTolerance) const
{
  double anAng = Angle (theOther);
  if (anAng < 0)
  {
    anAng = -anAng;
  }
  anAng = M_PI / 2.0 - anAng;
  if (anAng < 0)
  {
    anAng = -anAng;
  }
  return anAng <= theAngularTolerance;
}

// =======================================================================
// function : IsOpposite
// purpose  :
// =======================================================================
inline bool gp_Dir2d::IsOpposite (const gp_Dir2d& theOther,
                                              const double theAngularTolerance) const
{ 
  double anAng = Angle (theOther);
  if (anAng < 0)
  {
    anAng = -anAng;
  }
  return M_PI - anAng <= theAngularTolerance;
}

// =======================================================================
// function : IsParallel
// purpose  :
// =======================================================================
inline bool gp_Dir2d::IsParallel (const gp_Dir2d& theOther,
                                              const double theAngularTolerance) const
{
  double anAng = Angle (theOther);
  if (anAng < 0)
  {
    anAng = -anAng;
  }
  return anAng <= theAngularTolerance || M_PI - anAng <= theAngularTolerance;
}

// =======================================================================
// function : Rotate
// purpose  :
// =======================================================================
inline void gp_Dir2d::Rotate (const double theAng)
{
  gp_Trsf2d aT;
  aT.SetRotation (gp_Pnt2d (0.0, 0.0), theAng);
  coord.Multiply (aT.HVectorialPart());
}

double gp_Dir2d::Angle (const gp_Dir2d& Other) const
{
  //    Commentaires :
  //    Au dessus de 45 degres l'arccos donne la meilleur precision pour le
  //    calcul de l'angle. Sinon il vaut mieux utiliser l'arcsin.
  //    Les erreurs commises sont loin d'etre negligeables lorsque l'on est
  //    proche de zero ou de 90 degres.
  //    En 2D les valeurs angulaires sont comprises entre -PI et PI
  double Cosinus = coord.Dot   (Other.coord);
  double Sinus = coord.Crossed (Other.coord);
  if (Cosinus > -0.70710678118655 && Cosinus < 0.70710678118655) { 
    if (Sinus > 0.0) return   acos (Cosinus);
    else             return - acos (Cosinus);
  }
  else {
    if (Cosinus > 0.0)  return      asin (Sinus);
    else { 
      if (Sinus > 0.0) return  M_PI - asin (Sinus);
      else             return -M_PI - asin (Sinus);
    }
  }
}

void gp_Dir2d::Mirror (const gp_Ax2d& A2)
{
  const gp_XY& XY = A2.Direction().XY();
  double A = XY.X();
  double B = XY.Y();
  double X = coord.X();
  double Y = coord.Y();
  double M1 = 2.0 * A * B;
  double XX = ((2.0 * A * A) - 1.0) * X + M1 * Y;
  double YY = M1 * X + ((2.0 * B * B) - 1.0) * Y;
  coord.SetCoord(XX,YY);
}

void gp_Dir2d::Transform (const gp_Trsf2d& T)
{
  if (T.Form() == gp_Identity || T.Form() == gp_Translation)    { }
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

void gp_Dir2d::Mirror (const gp_Dir2d& V)
{
  const gp_XY& XY = V.coord;
  double A = XY.X();
  double B = XY.Y();
  double X = coord.X();
  double Y = coord.Y();
  double M1 = 2.0 * A * B;
  double XX = ((2.0 * A * A) - 1.0) * X + M1 * Y;
  double YY = M1 * X + ((2.0 * B * B) - 1.0) * Y;
  coord.SetCoord(XX,YY);
}

gp_Dir2d gp_Dir2d::Mirrored (const gp_Dir2d& V) const
{
  gp_Dir2d Vres = *this;
  Vres.Mirror (V);
  return Vres;
}

gp_Dir2d gp_Dir2d::Mirrored (const gp_Ax2d& A) const
{
  gp_Dir2d V = *this;
  V.Mirror (A);
  return V;
}
