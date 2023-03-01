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

// JCV 10/01/91 modifs suite a la deuxieme revue de projet
// AGV 03/04/07 bug correction: "pos" origin too far when A is very small

#define No_Standard_OutOfRange

#include <mobius/gp_Lin2d.hxx>

#include <mobius/gp.hxx>
#include <mobius/gp_Ax2d.hxx>
#include <mobius/gp_Dir2d.hxx>
#include <mobius/gp_Pnt2d.hxx>
#include <mobius/gp_Trsf2d.hxx>
#include <mobius/gp_Vec2d.hxx>

using namespace mobius::occ;

gp_Lin2d gp_Lin2d::Translated(const gp_Pnt2d& theP1, const gp_Pnt2d& theP2) const
{
  gp_Lin2d aL = *this;
  aL.pos.Translate (gp_Vec2d (theP1, theP2));
  return aL;
}

//=======================================================================
//function : gp_Lin2d
//purpose  : 
//=======================================================================
gp_Lin2d::gp_Lin2d (const double A,
                    const double B,
                    const double C)
{
  const double Norm2 = A * A + B * B;
  const gp_Pnt2d P (-A*C/Norm2, -B*C/Norm2);
  const gp_Dir2d V (-B, A);

//   gp_Pnt2d P;
//   double Norm = sqrt(A * A + B * B);
//   Standard_ConstructionError_Raise_if (Norm <= gp::Resolution(), " ");
//   double A1 = A/Norm;
//   double B1 = B/Norm;
//   double C1 = C/Norm;
//   gp_Dir2d V = gp_Dir2d (-B1, A1);
//   double AA1 = A1;
//   if (AA1 < 0) AA1 = - AA1;
//   if (AA1 > gp::Resolution()) P.SetCoord (-C1 / A1, 0.0);
//   else                        P.SetCoord (0.0, -C1 / B1);

  pos = gp_Ax2d(P, V);
}

//=======================================================================
//function : Mirror
//purpose  : 
//=======================================================================

void gp_Lin2d::Mirror (const gp_Pnt2d& P)
{ pos.Mirror(P);  }

//=======================================================================
//function : Mirrored
//purpose  : 
//=======================================================================

gp_Lin2d gp_Lin2d::Mirrored (const gp_Pnt2d& P)  const
{
  gp_Lin2d L = *this;
  L.pos.Mirror(P);
  return L;
}

//=======================================================================
//function : Mirror
//purpose  : 
//=======================================================================

void gp_Lin2d::Mirror (const gp_Ax2d& A)
{ pos.Mirror(A); }

//=======================================================================
//function : Mirrored
//purpose  : 
//=======================================================================

gp_Lin2d gp_Lin2d::Mirrored (const gp_Ax2d& A) const
{
  gp_Lin2d L = *this;
  L.pos.Mirror(A);
  return L;
}

