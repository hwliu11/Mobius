//-----------------------------------------------------------------------------
// Created on: 22 May 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef geom_PointOnLine_HeaderFile
#define geom_PointOnLine_HeaderFile

// Geometry includes
#include <mobius/geom_Line.h>

namespace mobius {

//! Checks whether a point belongs to a line.
class geom_PointOnLine
{
public:

  mobiusGeom_EXPORT bool
    operator()(const xyz&            P,
               const Ptr<geom_Line>& Line,
               const double          prec = 1.0e-6);

};

};

#endif
