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
// JCV 07/12/90 Modifs suite a l'introduction des classes XYZ et Mat dans gp

#define No_Standard_OutOfRange

#include <mobius/gp_Vec.hxx>

#include <mobius/gp.hxx>
#include <mobius/gp_Ax1.hxx>
#include <mobius/gp_Ax2.hxx>
#include <mobius/gp_Dir.hxx>
#include <mobius/gp_Pnt.hxx>
#include <mobius/gp_Trsf.hxx>
#include <mobius/gp_XYZ.hxx>

using namespace mobius::occ;

//=======================================================================
//function :  gp_Vec
// purpose :
//=======================================================================
inline gp_Vec::gp_Vec (const gp_Dir& theV)
{
  coord = theV.XYZ();
}

//=======================================================================
//function :  gp_Vec
// purpose :
//=======================================================================
inline gp_Vec::gp_Vec (const gp_Pnt& theP1, const gp_Pnt& theP2)
{
  coord = theP2.XYZ().Subtracted (theP1.XYZ());
}

//=======================================================================
//function :  IsNormal
// purpose :
//=======================================================================
inline bool gp_Vec::IsNormal (const gp_Vec& theOther, const double theAngularTolerance) const
{
  double anAng = M_PI / 2.0 - Angle (theOther);
  if (anAng < 0)
  {
    anAng = -anAng;
  }
  return  anAng <= theAngularTolerance;
}

//=======================================================================
//function :  Angle
// purpose :
//=======================================================================
inline double gp_Vec::Angle (const gp_Vec& theOther) const
{
  return (gp_Dir (coord)).Angle (theOther);
}

//=======================================================================
//function :  AngleWithRef
// purpose :
//=======================================================================
inline double gp_Vec::AngleWithRef (const gp_Vec& theOther, const gp_Vec& theVRef) const
{
  return (gp_Dir (coord)).AngleWithRef (theOther, theVRef);
}

//=======================================================================
//function :  Normalized
// purpose :
//=======================================================================
inline gp_Vec gp_Vec::Normalized() const
{
  double aD = coord.Modulus();
  gp_Vec aV = *this;
  aV.coord.Divide (aD);
  return aV;
}

//=======================================================================
//function :  Rotate
// purpose :
//=======================================================================
inline void gp_Vec::Rotate (const gp_Ax1& theA1, const double theAng)
{
  gp_Trsf aT;
  aT.SetRotation (theA1, theAng);
  coord.Multiply (aT.VectorialPart());
}

//=======================================================================
//function :  operator*
// purpose :
//=======================================================================
inline gp_Vec operator* (const double theScalar, const gp_Vec& theV)
{
  return theV.Multiplied(theScalar);
}

bool gp_Vec::IsEqual
(const gp_Vec& Other, 
 const double LinearTolerance,
 const double AngularTolerance) const
{
  if (Magnitude ()       <= LinearTolerance || 
      Other.Magnitude () <= LinearTolerance) {
    double val = Magnitude() - Other.Magnitude();
    if (val < 0) val = - val;
    return val <= LinearTolerance;
  }
  else {
    double val = Magnitude() - Other.Magnitude();
    if (val < 0) val = - val;
    return val <= LinearTolerance && Angle(Other) <= AngularTolerance;
  }
}    

void gp_Vec::Mirror (const gp_Vec& V)
{
  double D = V.coord.Modulus();
  if (D > gp::Resolution()) {
    const gp_XYZ& XYZ = V.coord;
    double A = XYZ.X() / D;
    double B = XYZ.Y() / D;
    double C = XYZ.Z() / D; 
    double M1 = 2.0 * A * B;
    double M2 = 2.0 * A * C;
    double M3 = 2.0 * B * C;
    double X = coord.X();
    double Y = coord.Y();
    double Z = coord.Z();
    coord.SetX(((2.0 * A * A) - 1.0) * X + M1 * Y + M2 * Z);
    coord.SetY(M1 * X + ((2.0 * B * B) - 1.0) * Y + M3 * Z);
    coord.SetZ(M2 * X + M3 * Y + ((2.0 * C * C) - 1.0) * Z);
  }
}

void gp_Vec::Mirror (const gp_Ax1& A1)
{
  const gp_XYZ& V = A1.Direction().XYZ();
  double A = V.X();
  double B = V.Y();
  double C = V.Z();
  double X = coord.X();
  double Y = coord.Y();
  double Z = coord.Z();
  double M1 = 2.0 * A * B;
  double M2 = 2.0 * A * C;
  double M3 = 2.0 * B * C;
  coord.SetX(((2.0 * A * A) - 1.0) * X + M1 * Y + M2 * Z);
  coord.SetY(M1 * X + ((2.0 * B * B) - 1.0) * Y + M3 * Z);
  coord.SetZ(M2 * X + M3 * Y + ((2.0 * C * C) - 1.0) * Z);
}

void gp_Vec::Mirror (const gp_Ax2& A2)
{
  gp_XYZ Z      = A2.Direction().XYZ();
  gp_XYZ MirXYZ = Z.Crossed (coord);
  if (MirXYZ.Modulus() <= gp::Resolution()) { coord.Reverse(); }
  else {
    Z.Cross (MirXYZ);
    Mirror (Z);
  }
}

void gp_Vec::Transform(const gp_Trsf& T)
{
  if (T.Form() == gp_Identity || T.Form() == gp_Translation) { }
  else if (T.Form() == gp_PntMirror) { coord.Reverse(); }
  else if (T.Form() == gp_Scale) { coord.Multiply (T.ScaleFactor()); }
  else { coord.Multiply (T.VectorialPart()); }
}

gp_Vec gp_Vec::Mirrored (const gp_Vec& V) const
{
  gp_Vec Vres = *this;
  Vres.Mirror (V);
  return Vres;
}

gp_Vec gp_Vec::Mirrored (const gp_Ax1& A1) const
{
  gp_Vec Vres = *this;
  Vres.Mirror (A1);
  return Vres;
}

gp_Vec gp_Vec::Mirrored (const gp_Ax2& A2) const
{
  gp_Vec Vres = *this;
  Vres.Mirror (A2);
  return Vres;
}
