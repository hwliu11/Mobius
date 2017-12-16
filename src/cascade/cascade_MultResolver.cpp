//-----------------------------------------------------------------------------
// Created on: 24 December 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

// Own include
#include <mobius/cascade_MultResolver.h>

//! Default constructor.
mobius::cascade_MultResolver::cascade_MultResolver()
{}

//! Default destructor.
mobius::cascade_MultResolver::~cascade_MultResolver()
{}

//! Resolves for the passed parameter.
//! \param u [in] knot value to resolve.
void mobius::cascade_MultResolver::Resolve(const double u)
{
  bool isFound = false;
  int foundIdx = -1;
  knot_multiset::elem foundStruct;
  for ( int i = 1; i <= Knots.Length(); ++i )
  {
    const knot_multiset::elem& knotWithMult = Knots.Value(i);
    if ( Abs(knotWithMult.u - u) < RealEpsilon() )
    {
      isFound = true;
      foundIdx = i;
      foundStruct = knotWithMult;
      break;
    }
  }
  if ( isFound )
  {
    foundStruct.m += 1;
    Knots.ChangeValue(foundIdx) = foundStruct;
  }
  else
  {
    foundStruct.u = u;
    foundStruct.m = 1;
    Knots.Append(foundStruct);
  }
}

//! \return knot array in OCCT form.
Handle(TColStd_HArray1OfReal)
  mobius::cascade_MultResolver::GetOpenCascadeKnots() const
{
  Handle(TColStd_HArray1OfReal) res = new TColStd_HArray1OfReal( 0, Knots.Length() - 1 );
  for ( int i = 0; i < Knots.Length(); ++i )
  {
    res->SetValue( i, Knots(i + 1).u );
  }
  return res;
}

//! \return multiplicity array in OCCT form.
Handle(TColStd_HArray1OfInteger)
  mobius::cascade_MultResolver::GetOpenCascadeMults() const
{
  Handle(TColStd_HArray1OfInteger) res = new TColStd_HArray1OfInteger( 0, Knots.Length() - 1 );
  for ( int i = 0; i < Knots.Length(); ++i )
  {
    res->SetValue( i, Knots(i + 1).m );
  }
  return res;
}
