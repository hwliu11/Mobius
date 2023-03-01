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

#define No_Standard_OutOfRange

#include <mobius/gp_Vec2d.hxx>

#include <mobius/gp.hxx>
#include <mobius/gp_Ax2d.hxx>
#include <mobius/gp_Dir2d.hxx>
#include <mobius/gp_Pnt2d.hxx>
#include <mobius/gp_Trsf2d.hxx>
#include <mobius/gp_XY.hxx>

using namespace mobius::occ;

//=======================================================================
//function :  gp_Vec2d
// purpose :
//=======================================================================
inline gp_Vec2d::gp_Vec2d (const gp_Dir2d& theV)
{
  coord = theV.XY();
}

//=======================================================================
//function :  gp_Vec2d
// purpose :
//=======================================================================
inline gp_Vec2d::gp_Vec2d (const gp_Pnt2d& theP1, const gp_Pnt2d& theP2)
{
  coord = theP2.XY().Subtracted (theP1.XY());
}

//=======================================================================
//function :  IsOpposite
// purpose :
//=======================================================================
inline bool gp_Vec2d::IsOpposite (const gp_Vec2d& theOther, const double theAngularTolerance) const
{
  double anAng = Angle (theOther);
  if (anAng < 0)
  {
    anAng = -anAng;
  }
  return M_PI - anAng <= theAngularTolerance;
}

//=======================================================================
//function :  IsParallel
// purpose :
//=======================================================================
inline bool gp_Vec2d::IsParallel (const gp_Vec2d& theOther, const double theAngularTolerance) const
{
  double anAng = Angle (theOther);
  if (anAng < 0)
  {
    anAng = -anAng;
  }
  return anAng <= theAngularTolerance || M_PI - anAng <= theAngularTolerance;
}

//=======================================================================
//function :  Normalized
// purpose :
//=======================================================================
inline gp_Vec2d gp_Vec2d::Normalized() const
{
  double aD = coord.Modulus();
  gp_Vec2d aV = *this;
  aV.coord.Divide (aD);
  return aV;
}

//=======================================================================
//function :  Rotate
// purpose :
//=======================================================================
inline void gp_Vec2d::Rotate (const double theAng)
{
  gp_Trsf2d aT;
  aT.SetRotation (gp_Pnt2d(0.0, 0.0), theAng);
  coord.Multiply (aT.VectorialPart());
}

//=======================================================================
//function :  operator*
// purpose :
//=======================================================================
inline gp_Vec2d operator* (const double theScalar,
                           const gp_Vec2d& theV)
{
  return theV.Multiplied (theScalar);
}


bool gp_Vec2d::IsEqual
(const gp_Vec2d& Other, 
 const double LinearTolerance,
 const double AngularTolerance) const
{
  const double theNorm = Magnitude();
  const double theOtherNorm = Other.Magnitude();
  double val = theNorm - theOtherNorm;
  if (val < 0.0) val = -val;
  // Check for equal lengths
  const bool isEqualLength = (val <= LinearTolerance);
  // Check for small vectors
  if (theNorm > LinearTolerance && theOtherNorm > LinearTolerance)
  {
    double Ang = Angle(Other);
    if (Ang < 0.0) Ang = -Ang;
    // Check for zero angle
    return isEqualLength && (Ang <= AngularTolerance);
  }
  return isEqualLength;
}    

double gp_Vec2d::Angle (const gp_Vec2d& Other) const
{
  //    Commentaires :
  //    Au dessus de 45 degres l'arccos donne la meilleur precision pour le
  //    calcul de l'angle. Sinon il vaut mieux utiliser l'arcsin.
  //    Les erreurs commises sont loin d'etre negligeables lorsque l'on est
  //    proche de zero ou de 90 degres.
  //    En 2D les valeurs angulaires sont comprises entre -PI et PI
  const double theNorm = Magnitude();
  const double theOtherNorm = Other.Magnitude();
  if (theNorm <= gp::Resolution() || theOtherNorm <= gp::Resolution())
    throw std::runtime_error("vector with null modulus");

  const double D = theNorm * theOtherNorm;
  const double Cosinus = coord.Dot   (Other.coord) / D;
  const double Sinus = coord.Crossed (Other.coord) / D;
  if (Cosinus > -0.70710678118655 && Cosinus < 0.70710678118655)
  {
    if (Sinus > 0.0)  return  acos (Cosinus);
    else              return -acos (Cosinus); 
  }
  else
  {
    if (Cosinus > 0.0) return        asin (Sinus);
    else
    { 
      if (Sinus > 0.0) return   M_PI - asin (Sinus);
      else             return - M_PI - asin (Sinus);
    }
  }  
}

void gp_Vec2d::Mirror (const gp_Ax2d& A1)
{
  const gp_XY& XY = A1.Direction().XY();
  double X = coord.X();
  double Y = coord.Y();
  double A = XY.X();
  double B = XY.Y();
  double M1 = 2.0 * A * B;
  coord.SetX(((2.0 * A * A) - 1.) * X + M1 * Y);
  coord.SetY(M1 * X + ((2. * B * B) - 1.0) * Y);
}

gp_Vec2d gp_Vec2d::Mirrored (const gp_Ax2d& A1) const
{
  gp_Vec2d Vres = *this;
  Vres.Mirror(A1);
  return Vres;                     
}

void gp_Vec2d::Transform (const gp_Trsf2d& T)
{
  if (T.Form() == gp_Identity || T.Form() == gp_Translation) { }
  else if (T.Form() == gp_PntMirror) coord.Reverse ();
  else if (T.Form() == gp_Scale)     coord.Multiply (T.ScaleFactor ());
  else                               coord.Multiply (T.VectorialPart ());
}

void gp_Vec2d::Mirror (const gp_Vec2d& V)
{
  const double D = V.coord.Modulus();
  if (D > gp::Resolution())
  {
    const gp_XY& XY = V.coord;
    double X = XY.X();
    double Y = XY.Y();
    double A = X / D;
    double B = Y / D;
    double M1 = 2.0 * A * B;
    coord.SetX(((2.0 * A * A) - 1.0) * X + M1 * Y);
    coord.SetY(M1 * X + ((2.0 * B * B) - 1.0) * Y);
  }
}

gp_Vec2d gp_Vec2d::Mirrored (const gp_Vec2d& V) const
{
  gp_Vec2d Vres = *this;
  Vres.Mirror(V);
  return Vres;                     
}
