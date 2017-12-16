//-----------------------------------------------------------------------------
// Created on: 24 December 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef cascade_MultResolver_HeaderFile
#define cascade_MultResolver_HeaderFile

// Cascade includes
#include <mobius/cascade.h>

// BSpl includes
#include <mobius/bspl_KnotMultiset.h>

// OCCT includes
#include <NCollection_Sequence.hxx>
#include <TColStd_HArray1OfReal.hxx>
#include <TColStd_HArray1OfInteger.hxx>

namespace mobius {

//! The way how knots are represented in Mobius is different from OCCT.
//! OCCT stores each knot value just once (without repetitions), however,
//! it requires additional array with multiplicities. E.g. U = (0f, 0f, 1f, 2f, 2f)
//! is represented by two arrays in OCCT: (0f, 1f, 2f) for the parameter values
//! and (2, 1, 2) for their multiplicities. Mobius is more straightforward
//! concerning this. This auxiliary tool performs necessary conversion
//! from Mobius notation to OCCT one.
class cascade_MultResolver
{
// Members:
public:

  NCollection_Sequence<bspl_KnotMultiset::elem> Knots; //!< Knots being processed.

public:

  mobiusCascade_EXPORT
    cascade_MultResolver();

  mobiusCascade_EXPORT virtual
    ~cascade_MultResolver();

public:

  mobiusCascade_EXPORT void
    Resolve(const double u);

  mobiusCascade_EXPORT Handle(TColStd_HArray1OfReal)
    GetOpenCascadeKnots() const;

  mobiusCascade_EXPORT Handle(TColStd_HArray1OfInteger)
    GetOpenCascadeMults() const;

};

};

#endif
