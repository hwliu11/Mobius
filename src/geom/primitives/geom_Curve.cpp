//-----------------------------------------------------------------------------
// Created on: 05 August 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

// Own include
#include <mobius/geom_Curve.h>

//! Constructor accepting transformations to apply.
//! \param tChain [in] associated transformations.
mobius::geom_Curve::geom_Curve(const core_IsoTransformChain& tChain)
: geom_Geometry(tChain)
{}

//! Destructor.
mobius::geom_Curve::~geom_Curve()
{}
