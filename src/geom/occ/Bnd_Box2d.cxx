// Created on: 1991-10-30
// Created by: Modelisation
// Copyright (c) 1991-1999 Matra Datavision
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


#include <mobius/Bnd_Box2d.hxx>

#include <mobius/gp.hxx>
#include <mobius/gp_Dir2d.hxx>
#include <mobius/gp_Trsf2d.hxx>
#include <mobius/Standard_Real.hxx>

using namespace mobius::occ;

//-- #include <Precision.hxx> Precision::Infinite() -> 1e+100
//=======================================================================
//function : Update
//purpose  : 
//=======================================================================
void Bnd_Box2d::Update (const double x, const double y, 
			const double X, const double Y)
{
  if (Flags & VoidMask) {
    Xmin = x;
    Ymin = y;
    Xmax = X;
    Ymax = Y;
    Flags &= ~VoidMask;
  }
  else {
    if (!(Flags & XminMask) && (x < Xmin)) Xmin = x;
    if (!(Flags & XmaxMask) && (X > Xmax)) Xmax = X;
    if (!(Flags & YminMask) && (y < Ymin)) Ymin = y;
    if (!(Flags & YmaxMask) && (Y > Ymax)) Ymax = Y;
  }
}

//=======================================================================
//function : Update
//purpose  : 
//=======================================================================

void Bnd_Box2d::Update (const double X, const double Y)
{
  if (Flags & VoidMask) {
    Xmin = X;
    Ymin = Y;
    Xmax = X;
    Ymax = Y;
    Flags &= ~VoidMask;
  }
  else {
    if      (!(Flags & XminMask) && (X < Xmin)) Xmin = X;
    else if (!(Flags & XmaxMask) && (X > Xmax)) Xmax = X;
    if      (!(Flags & YminMask) && (Y < Ymin)) Ymin = Y;
    else if (!(Flags & YmaxMask) && (Y > Ymax)) Ymax = Y;
  }
}

//=======================================================================
//function : Get
//purpose  : 
//=======================================================================

void Bnd_Box2d::Get (double& x, double& y,
		     double& Xm, double& Ym) const
{
  if(Flags & VoidMask)
    throw std::runtime_error("Bnd_Box is void");
  double pinf = 1e+100; //-- Precision::Infinite();
  if (Flags & XminMask) x = -pinf;
  else                  x =  Xmin-Gap;
  if (Flags & XmaxMask) Xm =  pinf;
  else                  Xm =  Xmax+Gap;
  if (Flags & YminMask) y = -pinf;
  else                  y =  Ymin-Gap;
  if (Flags & YmaxMask) Ym =  pinf;
  else                  Ym =  Ymax+Gap;
}

//=======================================================================
//function : Transformed
//purpose  : 
//=======================================================================

Bnd_Box2d Bnd_Box2d::Transformed (const gp_Trsf2d& T) const
{
  gp_TrsfForm F = T.Form();
  Bnd_Box2d newb(*this);
  if ( IsVoid() ) return newb;

  if      (F == gp_Identity) {}
  else if (F == gp_Translation) {
    double DX,DY;
    (T.TranslationPart()).Coord(DX,DY);
    if (!(Flags & XminMask))  newb.Xmin += DX;
    if (!(Flags & XmaxMask))  newb.Xmax += DX;
    if (!(Flags & YminMask))  newb.Ymin += DY;
    if (!(Flags & YmaxMask))  newb.Ymax += DY;
  }
  else {
    gp_Pnt2d P[4];
    bool Vertex[4];
    int i;
    Vertex[0] = true;
    Vertex[1] = true;
    Vertex[2] = true;
    Vertex[3] = true;
    gp_Dir2d D[6];
//    int vertices = 0;
    int directions = 0;

    if (Flags & XminMask) {
      D[directions].SetCoord(-1., 0.);
      directions++;
      Vertex[0] = Vertex[2] = false;
    }
    if (Flags & XmaxMask) {
      D[directions].SetCoord( 1., 0.);
      directions++;
      Vertex[1] = Vertex[3] = false;
    }
    if (Flags & YminMask) {
      D[directions].SetCoord( 0.,-1.);
      directions++;
      Vertex[0] = Vertex[1] = false;
    }
    if (Flags & YmaxMask) {
      D[directions].SetCoord( 0., 1.);
      directions++;
      Vertex[2] = Vertex[3] = false;
    }

    newb.SetVoid();

    for (i = 0; i < directions; i++) {
      D[i].Transform(T);
      newb.Add(D[i]);
    }
    P[0].SetCoord(Xmin,Ymin);
    P[1].SetCoord(Xmax,Ymin);
    P[2].SetCoord(Xmin,Ymax);
    P[3].SetCoord(Xmax,Ymax);
    if (Vertex[0]) {
      P[0].Transform(T);
      newb.Add(P[0]);
    }
    if (Vertex[1]) {
      P[1].Transform(T);
      newb.Add(P[1]);
    }
    if (Vertex[2]) {
      P[2].Transform(T);
      newb.Add(P[2]);
    }
    if (Vertex[3]) {
      P[3].Transform(T);
      newb.Add(P[3]);
    }
    newb.Gap=Gap;
  }
  return newb;
}

//=======================================================================
//function : Add
//purpose  : 
//=======================================================================

void Bnd_Box2d::Add (const Bnd_Box2d& Other)
{
  if (IsWhole()) return;
  else if (Other.IsVoid()) return; 
  else if (Other.IsWhole()) SetWhole();
  else if (IsVoid()) (*this) = Other;
  else
  {
    if ( ! IsOpenXmin() )
    {
      if (Other.IsOpenXmin()) OpenXmin();
      else if (Xmin > Other.Xmin) Xmin = Other.Xmin;
    }
    if ( ! IsOpenXmax() )
    {
      if (Other.IsOpenXmax()) OpenXmax();
      else if (Xmax < Other.Xmax) Xmax = Other.Xmax;
    }
    if ( ! IsOpenYmin() )
    {
      if (Other.IsOpenYmin()) OpenYmin();
      else if (Ymin > Other.Ymin) Ymin = Other.Ymin;
    }
    if ( ! IsOpenYmax() )
    {
      if (Other.IsOpenYmax()) OpenYmax();
      else if (Ymax < Other.Ymax) Ymax = Other.Ymax;
    }
    Gap = Max (Gap, Other.Gap);
  }
}

//=======================================================================
//function : Add
//purpose  : 
//=======================================================================

void Bnd_Box2d::Add (const gp_Dir2d& D)
{
  double DX = D.X();
  double DY = D.Y();

  if (DX < -RealEpsilon()) 
    OpenXmin();
  else if (DX > RealEpsilon()) 
    OpenXmax();

  if (DY < -RealEpsilon())
    OpenYmin();
  else if (DY > RealEpsilon())
    OpenYmax();
}

//=======================================================================
//function : IsOut
//purpose  : 
//=======================================================================

bool Bnd_Box2d::IsOut (const gp_Pnt2d& P) const
{
  if        (IsWhole())  return false;
  else if   (IsVoid())   return true;
  else {
    double X = P.X();
    double Y = P.Y();
    if      (!(Flags & XminMask) && (X < (Xmin-Gap))) return true;
    else if (!(Flags & XmaxMask) && (X > (Xmax+Gap))) return true;
    else if (!(Flags & YminMask) && (Y < (Ymin-Gap))) return true;
    else if (!(Flags & YmaxMask) && (Y > (Ymax+Gap))) return true;
    else return false;
  }
}

//=======================================================================
//function : IsOut
//purpose  :
//=======================================================================

bool Bnd_Box2d::IsOut(const gp_Lin2d& theL) const
{
  if (IsWhole())
  {
    return false;
  }
  if (IsVoid())
  {
    return true;
  }
  double aXMin, aXMax, aYMin, aYMax;
  Get(aXMin, aYMin, aXMax, aYMax);

  gp_XY aCenter((aXMin + aXMax) / 2, (aYMin + aYMax) / 2);
  gp_XY aHeigh(Abs(aXMax - aCenter.X()), Abs(aYMax - aCenter.Y()));

  const double aProd[3] = {
    theL.Direction().XY() ^ (aCenter - theL.Location().XY()),
    theL.Direction().X() * aHeigh.Y(),
    theL.Direction().Y() * aHeigh.X()
  };
  bool aStatus = (Abs(aProd[0]) > (Abs(aProd[1]) + Abs(aProd[2])));
  return aStatus;
}

//=======================================================================
//function : IsOut
//purpose  :
//=======================================================================

bool Bnd_Box2d::IsOut(const gp_Pnt2d& theP0, const gp_Pnt2d& theP1) const
{
  if (IsWhole())
  {
    return false;
  }
  if (IsVoid())
  {
    return true;
  }
  
  bool aStatus = true;
  double aLocXMin, aLocXMax, aLocYMin, aLocYMax;
  Get(aLocXMin, aLocYMin, aLocXMax, aLocYMax);

  //// Intersect the line containing the segment.
  const gp_XY aSegDelta(theP1.XY() - theP0.XY());

  gp_XY aCenter((aLocXMin + aLocXMax) / 2, (aLocYMin + aLocYMax) / 2);
  gp_XY aHeigh(Abs(aLocXMax - aCenter.X()), Abs(aLocYMax - aCenter.Y()));

  const double aProd[3] = {
    aSegDelta ^ (aCenter - theP0.XY()),
    aSegDelta.X() * aHeigh.Y(),
    aSegDelta.Y() * aHeigh.X()
  };

  if((Abs(aProd[0]) <= (Abs(aProd[1]) + Abs(aProd[2]))))
  {
    // Intersection with line detected; check the segment as bounding box
    const gp_XY aHSeg(0.5 * aSegDelta.X(), 0.5 * aSegDelta.Y());
    const gp_XY aHSegAbs(Abs(aHSeg.X()), Abs(aHSeg.Y()));
    aStatus = ((Abs((theP0.XY() + aHSeg - aCenter).X()) >
      (aHeigh + aHSegAbs).X()) || (Abs((theP0.XY() + aHSeg - aCenter).Y()) >
      (aHeigh + aHSegAbs).Y()));
  }
  return aStatus;
}

//=======================================================================
//function : IsOut
//purpose  : 
//=======================================================================

bool Bnd_Box2d::IsOut (const Bnd_Box2d& Other) const
{
  if        (IsWhole())  return false;
  else if   (IsVoid())   return true;
  else if   (Other.IsWhole())  return false;
  else if   (Other.IsVoid())   return true;
  else {
    double OXmin,OXmax,OYmin,OYmax;
    Other.Get(OXmin,OYmin,OXmax,OYmax);
    if      (!(Flags & XminMask) && (OXmax < (Xmin-Gap))) return true;
    else if (!(Flags & XmaxMask) && (OXmin > (Xmax+Gap))) return true;
    else if (!(Flags & YminMask) && (OYmax < (Ymin-Gap))) return true;
    else if (!(Flags & YmaxMask) && (OYmin > (Ymax+Gap))) return true;
  }
  return false;
}

//=======================================================================
//function : Dump
//purpose  : 
//=======================================================================

void Bnd_Box2d::Dump () const
{
  std::cout << "Box2d : ";
  if      (IsVoid())  std::cout << "Void";
  else if (IsWhole()) std::cout << "Whole";
  else {
    std::cout << "\n Xmin : ";
    if (IsOpenXmin()) std::cout << "Infinite";
    else              std::cout << Xmin;
    std::cout << "\n Xmax : ";
    if (IsOpenXmax()) std::cout << "Infinite";
    else              std::cout << Xmax;
    std::cout << "\n Ymin : ";
    if (IsOpenYmin()) std::cout << "Infinite";
    else              std::cout << Ymin;
    std::cout << "\n Ymax : ";
    if (IsOpenYmax()) std::cout << "Infinite";
    else              std::cout << Ymax;
  }
  std::cout << "\n Gap : " << Gap;
  std::cout << "\n";
}
