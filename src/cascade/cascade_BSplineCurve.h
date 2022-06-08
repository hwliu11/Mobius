//-----------------------------------------------------------------------------
// Created on: 05 August 2013
//-----------------------------------------------------------------------------
// Copyright (c) 2013-present, Sergey Slyadnev
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

#ifndef cascade_BSplineCurve_HeaderFile
#define cascade_BSplineCurve_HeaderFile

// Cascade includes
#include <mobius/cascade.h>

// OCCT includes
#include <GeomAbs_Shape.hxx>

namespace mobius {

//! \ingroup MOBIUS_CASCADE
//!
//! Converter between OpenCascade and Mobius for B-curves.
class cascade_BSplineCurve
{
public:

  mobiusCascade_EXPORT
    cascade_BSplineCurve(const t_ptr<t_bcurve>& mobiusCurve);

  mobiusCascade_EXPORT
    cascade_BSplineCurve(const Handle(Geom_BSplineCurve)& occtCurve);

  mobiusCascade_EXPORT
    ~cascade_BSplineCurve();

public:

  mobiusCascade_EXPORT void
    ReApproxMobius(const double        theTol3d,
                   const GeomAbs_Shape theOrder,
                   const int           theMaxSegments,
                   const int           theMaxDegree);

  mobiusCascade_EXPORT void
    DirectConvert();

public:

  mobiusCascade_EXPORT const t_ptr<t_bcurve>&
    GetMobiusCurve() const;

  mobiusCascade_EXPORT const Handle(Geom_BSplineCurve)&
    GetOpenCascadeCurve() const;

  mobiusCascade_EXPORT bool
    IsDone() const;

  mobiusCascade_EXPORT double
    MaxError() const;

protected:

  mobiusCascade_EXPORT void
    convertToOpenCascade();

  mobiusCascade_EXPORT void
    convertToMobius();

private:

  //! Mobius curve.
  t_ptr<t_bcurve> m_mobiusCurve;

  //! OCCT curve.
  Handle(Geom_BSplineCurve) m_occtCurve;

  //! Maximum achieved approximation error.
  double m_fMaxError;

  //! Indicates whether conversion is done or not.
  bool m_bIsDone;

};

}

#endif
