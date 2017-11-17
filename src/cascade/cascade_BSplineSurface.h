//-----------------------------------------------------------------------------
// Created on: 24 December 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef cascade_BSplineSurface_HeaderFile
#define cascade_BSplineSurface_HeaderFile

// Cascade includes
#include <mobius/cascade.h>

// Geom includes
#include <mobius/geom_BSplineSurface.h>

// OCCT includes
#include <Geom_BSplineSurface.hxx>

namespace mobius {

//! Bridge for conversions between Mobius and OCCT B-surfaces.
class cascade_BSplineSurface
{
public:

  mobiusCascade_EXPORT static Handle(Geom_BSplineSurface)
    FromMobius(const Ptr<bsurf>& source);

};

};

#endif
