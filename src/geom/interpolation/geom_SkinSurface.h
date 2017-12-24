//-----------------------------------------------------------------------------
// Created on: 09 March 2015
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

#ifndef geom_SkinSurface_HeaderFile
#define geom_SkinSurface_HeaderFile

// Geometry includes
#include <mobius/geom_BSplineCurve.h>
#include <mobius/geom_BSplineSurface.h>

// BSpl includes
#include <mobius/bspl_KnotsSelection.h>
#include <mobius/bspl_ParamsSelection.h>

// STL includes
#include <vector>

namespace mobius {

//! Creates B-surface passing through the given series of compatible (!)
//! B-curves.
class geom_SkinSurface
{
public:

  //! Error codes.
  enum ErrCode
  {
    ErrCode_NoError = 0,
    ErrCode_NotInitialized,
    ErrCode_NotDone,
    ErrCode_NotEnoughCurves,
    ErrCode_NullCurvePassed,
    ErrCode_NotCompatibleCurves_Degree,
    ErrCode_NotCompatibleCurves_Knots,
    ErrCode_BadVDegree,
    ErrCode_CannotSelectParameters,
    ErrCode_CannotSelectKnots,
    ErrCode_CannotInterpolateIsoU
  };

public:

  mobiusGeom_EXPORT
    geom_SkinSurface();

  mobiusGeom_EXPORT
    geom_SkinSurface(const std::vector< Ptr<bcurve> >& curves,
                     const int                         deg_V,
                     const bool                        unifyCurves);

public:

  mobiusGeom_EXPORT void
    Init(const std::vector< Ptr<bcurve> >& curves,
         const int                         deg_V,
         const bool                        unifyCurves);

  mobiusGeom_EXPORT void
    Perform();

public:

  //! Accessor for error code.
  //! \return error code.
  int ErrorCode() const
  {
    return m_errCode;
  }

  //! Accessor for the resulting surface.
  //! \return interpolant surface.
  const Ptr<bsurf>& Result() const
  {
    return m_surface;
  }

// For maintenance:
public:

  std::vector< Ptr<bcurve> > IsoU_Curves;

private:

  std::vector< Ptr<bcurve> > m_curves;  //!< Curves to interpolate.
  int                        m_iDeg_V;  //!< V-degree of interpolant surface.
  bool                       m_bUnify;  //!< Indicates whether to unify curves.
  ErrCode                    m_errCode; //!< Error code.
  Ptr<bsurf>                 m_surface; //!< Interpolant surface.

};

};

#endif
