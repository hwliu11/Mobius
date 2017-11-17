//-----------------------------------------------------------------------------
// Created on: 22 May 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

// Own include
#include <mobius/geom_PointOnLine.h>

//! Performs point-line classification.
//! \param P    [in] point to check.
//! \param Line [in] line to check.
//! \param prec [in] precision (tolerance around line).
//! \return true if the point belongs to the line, false -- otherwise.
bool mobius::geom_PointOnLine::operator()(const xyz&            P,
                                          const Ptr<geom_Line>& Line,
                                          const double          prec)
{
  // Line properties
  const xyz& Dir = Line->Dir();
  const xyz& Ori = Line->Origin();

  // Vector from line's origin to P
  xyz Ori_P = P - Ori;
  const double Ori_P_mod = Ori_P.Modulus();
  if ( Ori_P_mod < prec)
    return true;

  // Calculate angle between directions
  const double alpha = Dir.Angle(Ori_P);

  // Calculate distance from P to line
  const double h = Ori_P_mod*sin(alpha);

  // Classify
  if ( h < prec )
    return true;

  return false;
}
