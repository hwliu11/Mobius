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

#include <mobius/gp_Circ.hxx>

#include <mobius/gp_Ax1.hxx>
#include <mobius/gp_Ax2.hxx>
#include <mobius/gp_Pnt.hxx>
#include <mobius/gp_Trsf.hxx>
#include <mobius/gp_Vec.hxx>

using namespace mobius::occ;

Standard_Real gp_Circ::SquareDistance(const gp_Pnt& P) const
{
  gp_Vec V(Location(), P);
  Standard_Real x = V.Dot(pos.XDirection());
  Standard_Real y = V.Dot(pos.YDirection());
  Standard_Real z = V.Dot(pos.Direction());
  Standard_Real t = sqrt(x * x + y * y) - radius;
  return (t * t + z * z);
}

void gp_Circ::Mirror (const gp_Pnt& P)
{ pos.Mirror(P); }

gp_Circ gp_Circ::Mirrored (const gp_Pnt& P) const
{
  gp_Circ C = *this;
  C.pos.Mirror (P);
  return C; 
}

void gp_Circ::Mirror (const gp_Ax1& A1)
{ pos.Mirror(A1); }

gp_Circ gp_Circ::Mirrored (const gp_Ax1& A1) const
{
  gp_Circ C = *this;
  C.pos.Mirror (A1);
  return C; 
}

void gp_Circ::Mirror (const gp_Ax2& A2)
{ pos.Mirror(A2); }

gp_Circ gp_Circ::Mirrored (const gp_Ax2& A2) const
{
  gp_Circ C = *this;
  C.pos.Mirror (A2);
  return C; 
}

