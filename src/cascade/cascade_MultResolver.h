//-----------------------------------------------------------------------------
// Created on: 24 December 2014
//-----------------------------------------------------------------------------
// Copyright (c) 2017, Sergey Slyadnev
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//    * Neither the name of Sergey Slyadnev nor the
//      names of all contributors may be used to endorse or promote products
//      derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
//! is represented by two arrays in OCCT: (0f, 1f, 2f) for the knot values
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
