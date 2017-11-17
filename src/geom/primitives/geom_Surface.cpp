//-----------------------------------------------------------------------------
// Created on: 10 June 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

// Own include
#include <mobius/geom_Surface.h>

//! Constructor accepting transformations to apply.
//! \param tChain [in] transformations to apply.
mobius::geom_Surface::geom_Surface(const core_IsoTransformChain& tChain)
: geom_Geometry(tChain)
{}

//! Destructor.
mobius::geom_Surface::~geom_Surface()
{}
