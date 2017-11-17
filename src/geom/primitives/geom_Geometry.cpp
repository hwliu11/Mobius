//-----------------------------------------------------------------------------
// Created on: 23 May 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

// Own include
#include <mobius/geom_Geometry.h>

//! Default constructor.
//! \param tChain [in] transformations to apply.
mobius::geom_Geometry::geom_Geometry(const core_IsoTransformChain& tChain)
: core_OBJECT(),
  m_tChain(tChain)
{}

//! Destructor.
mobius::geom_Geometry::~geom_Geometry()
{}
