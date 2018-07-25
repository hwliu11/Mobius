//-----------------------------------------------------------------------------
// Created on: 26 October 2013
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

#ifndef geom_InterpolateCurve_HeaderFile
#define geom_InterpolateCurve_HeaderFile

// Geometry includes
#include <mobius/geom_BSplineCurve.h>

// BSpl includes
#include <mobius/bspl_KnotsSelection.h>
#include <mobius/bspl_ParamsSelection.h>

// STL includes
#include <vector>

namespace mobius {

//! Interpolates B-curve over the given collection of points.
class geom_InterpolateCurve
{
public:

  //! Error codes.
  enum ErrCode
  {
    ErrCode_NoError = 0,
    ErrCode_NotInitialized,
    ErrCode_NotDone,
    ErrCode_CannotSelectParameters,
    ErrCode_CannotSelectKnots,
    ErrCode_InterpolationFailed
  };

public:

  mobiusGeom_EXPORT
    geom_InterpolateCurve();

  mobiusGeom_EXPORT
    geom_InterpolateCurve(const std::vector<xyz>&    points,
                          const int                  deg,
                          const bspl_ParamsSelection paramsType,
                          const bspl_KnotsSelection  knotsType);

  mobiusGeom_EXPORT
    geom_InterpolateCurve(const std::vector<xyz>& points,
                          const int               deg,
                          double*                 pParams,
                          const int               n,
                          double*                 pU,
                          const int               m);

public:

  mobiusGeom_EXPORT void
    Init(const std::vector<xyz>&    points,
         const int                  deg,
         const bspl_ParamsSelection paramsType,
         double*                    pU,
         const int                  m);

  mobiusGeom_EXPORT void
    Init(const std::vector<xyz>& points,
         const int               deg,
         double*                 pParams,
         const int               n,
         double*                 pU,
         const int               m);

  mobiusGeom_EXPORT void
    Init(const std::vector<xyz>&    points,
         const int                  deg,
         const bspl_ParamsSelection paramsType,
         const bspl_KnotsSelection  knotsType);

  mobiusGeom_EXPORT void
    Init(const std::vector<xyz>&    points,
         const xyz&                 D0,
         const xyz&                 Dn,
         const int                  deg,
         const bspl_ParamsSelection paramsType,
         const bspl_KnotsSelection  knotsType);

  mobiusGeom_EXPORT void
    Init(const std::vector<xyz>&    points,
         const xyz&                 D0,
         const xyz&                 Dn,
         const xyz&                 D20,
         const xyz&                 D2n,
         const int                  deg,
         const bspl_ParamsSelection paramsType,
         const bspl_KnotsSelection  knotsType);

  mobiusGeom_EXPORT void
    Perform();

public:

  mobiusGeom_EXPORT static bool
    Interp(const std::vector<xyz>& points,
           const int               n,
           const int               p,
           const double*           params,
           const double*           U,
           const int               m,
           const bool              has_start_deriv,
           const bool              has_end_deriv,
           const bool              has_start_deriv2,
           const bool              has_end_deriv2,
           const xyz&              D0,
           const xyz&              Dn,
           const xyz&              D20,
           const xyz&              D2n,
           ptr<bcurve>&          crv);

public:

  //! Accessor for error code.
  //! \return error code.
  int ErrorCode() const
  {
    return m_errCode;
  }

  //! Accessor for the resulting curve.
  //! \return interpolant curve.
  const ptr<bcurve>& Result() const
  {
    return m_curve;
  }

protected:

  int  last_index_poles() const;
  int  last_index_knots() const;
  bool has_start_deriv()  const;
  bool has_end_deriv()    const;
  bool has_start_deriv2() const;
  bool has_end_deriv2()   const;

protected:

  static int dimension(const int  n,
                       const bool has_start_deriv,
                       const bool has_end_deriv,
                       const bool has_start_deriv2,
                       const bool has_end_deriv2);

private:

  int                  m_iDeg;       //!< Degree of interpolant curve.
  ErrCode              m_errCode;    //!< Error code.
  std::vector<xyz>     m_points;     //!< Points to interpolate.
  xyz                  m_D0;         //!< Derivative D1 at the first point.
  xyz                  m_Dn;         //!< Derivative D1 at the last point.
  xyz                  m_D20;        //!< Derivative D2 at the first point.
  xyz                  m_D2n;        //!< Derivative D2 at the last point.
  bspl_ParamsSelection m_paramsType; //!< Parameterization type.
  double*              m_pParams;    //!< Parameters externally defined for interpolation.
  int                  m_iNumParams; //!< Number of parameters.
  bspl_KnotsSelection  m_knotsType;  //!< Knots selection type.
  double*              m_pU;         //!< Knot vector externally defined for interpolation.
  int                  m_iNumKnots;  //!< Number of knots.
  ptr<bcurve>          m_curve;      //!< Interpolant curve.

};

};

#endif
