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

// Geom includes
#include <mobius/geom_BSplineCurve.h>

// Core includes
#include <mobius/core_OPERATOR.h>

// BSpl includes
#include <mobius/bspl_KnotsSelection.h>
#include <mobius/bspl_ParamsSelection.h>

// STL includes
#include <vector>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Interpolates B-curve over the given collection of points. See
//! algorithm A9.1 from The NURBS Book.
class geom_InterpolateCurve : public core_OPERATOR
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

  //! Default ctor.
  //! \param[in] progress progress notifier.
  //! \param[in] plotter  imperative plotter.
  mobiusGeom_EXPORT
    geom_InterpolateCurve(core_ProgressEntry progress = NULL,
                          core_PlotterEntry  plotter  = NULL);

  //! Complete ctor.
  //! \param[in] points     data points to interpolate.
  //! \param[in] deg        degree of B-spline functions to use for blending.
  //! \param[in] paramsType strategy for choosing interpolant parameters in
  //!                       the data points.
  //! \param[in] knotsType  strategy for choosing knot vector for interpolant
  //!                       B-splines.
  //! \param[in] progress   progress notifier.
  //! \param[in] plotter    imperative plotter.
  mobiusGeom_EXPORT
    geom_InterpolateCurve(const std::vector<t_xyz>&  points,
                          const int                  deg,
                          const bspl_ParamsSelection paramsType,
                          const bspl_KnotsSelection  knotsType,
                          core_ProgressEntry         progress = NULL,
                          core_PlotterEntry          plotter  = NULL);

  //! Complete constructor accepting the manually defined paraneters and
  //! knot vector.
  //! \param[in] points   data points to interpolate.
  //! \param[in] deg      degree of B-spline functions to use for blending.
  //! \param[in] pParams  manually defined interpolation parameters.
  //! \param[in] n        0-based index of the last parameter.
  //! \param[in] pU       manually defined knot vector.
  //! \param[in] m        0-based index of the last knot in the knot vector.
  //! \param[in] progress progress notifier.
  //! \param[in] plotter  imperative plotter.
  mobiusGeom_EXPORT
    geom_InterpolateCurve(const std::vector<t_xyz>& points,
                          const int                 deg,
                          double*                   pParams,
                          const int                 n,
                          double*                   pU,
                          const int                 m,
                          core_ProgressEntry        progress = NULL,
                          core_PlotterEntry         plotter  = NULL);

public:

  //! Initializes interpolation tool.
  //! \param[in] points     data points to interpolate.
  //! \param[in] deg        degree of B-spline functions to use for blending.
  //! \param[in] paramsType strategy for choosing interpolant parameters in
  //!                       the data points.
  //! \param[in] pU         raw pointer to the knot vector.
  //! \param[in] m          knot vector size minus one.
  mobiusGeom_EXPORT void
    Init(const std::vector<t_xyz>&  points,
         const int                  deg,
         const bspl_ParamsSelection paramsType,
         double*                    pU,
         const int                  m);

  //! Initializes interpolation tool.
  //! \param[in] points  data points to interpolate.
  //! \param[in] deg     degree of B-spline functions to use for blending.
  //! \param[in] pParams manually defined interpolation parameters.
  //! \param[in] n       0-based index of the last parameter.
  //! \param[in] pU      manually defined knot vector.
  //! \param[in] m       0-based index of the last knot in the knot vector.
  mobiusGeom_EXPORT void
    Init(const std::vector<t_xyz>& points,
         const int                 deg,
         double*                   pParams,
         const int                 n,
         double*                   pU,
         const int                 m);

  //! Initializes interpolation tool.
  //! \param[in] points  data points to interpolate.
  //! \param[in] deg     degree of B-spline functions to use for blending.
  //! \param[in] paramsType strategy for choosing interpolant parameters in
  //!                       the data points.
  //! \param[in] knotsType  strategy for choosing knot vector for interpolant
  //!                       B-splines.
  mobiusGeom_EXPORT void
    Init(const std::vector<t_xyz>&  points,
         const int                  deg,
         const bspl_ParamsSelection paramsType,
         const bspl_KnotsSelection  knotsType);

  //! Initializes interpolation tool.
  //! \param[in] points     data points to interpolate.
  //! \param[in] D0         derivative D1 at the first point.
  //! \param[in] Dn         derivative D1 at the last point.
  //! \param[in] deg        degree of B-spline functions to use for blending.
  //! \param[in] paramsType strategy for choosing interpolant parameters in
  //!                       the data points.
  //! \param[in] knotsType  strategy for choosing knot vector for interpolant
  //!                       B-splines.
  mobiusGeom_EXPORT void
    Init(const std::vector<t_xyz>&  points,
         const t_xyz&               D0,
         const t_xyz&               Dn,
         const int                  deg,
         const bspl_ParamsSelection paramsType,
         const bspl_KnotsSelection  knotsType);

  //! Initializes interpolation tool.
  //! \param[in] points     data points to interpolate.
  //! \param[in] D0         derivative D1 at the first point.
  //! \param[in] Dn         derivative D1 at the last point.
  //! \param[in] D20        derivative D2 at the first point.
  //! \param[in] D2n        derivative D2 at the last point.
  //! \param[in] deg        degree of B-spline functions to use for blending.
  //! \param[in] paramsType strategy for choosing interpolant parameters in
  //!                       the data points.
  //! \param[in] knotsType  strategy for choosing knot vector for interpolant
  //!                       B-splines.
  mobiusGeom_EXPORT void
    Init(const std::vector<t_xyz>&  points,
         const t_xyz&               D0,
         const t_xyz&               Dn,
         const t_xyz&               D20,
         const t_xyz&               D2n,
         const int                  deg,
         const bspl_ParamsSelection paramsType,
         const bspl_KnotsSelection  knotsType);

public:

  //! Performs interpolation.
  //! \return true in case of success, false -- otherwise.
  mobiusGeom_EXPORT bool
    Perform();

public:

  //! Interpolation kernel.
  //! \param[in]  points           reper points.
  //! \param[in]  n                index of the last pole (0-based).
  //! \param[in]  p                B-spline degree.
  //! \param[in]  params           parameters of the reper points.
  //! \param[in]  pU               knot vector.
  //! \param[in]  m                index of the last knot (0-based).
  //! \param[in]  has_start_deriv  indicates whether start derivative D1 is specified.
  //! \param[in]  has_end_deriv    indicates whether end derivative D1 is specified.
  //! \param[in]  has_start_deriv2 indicates whether start derivative D2 is specified.
  //! \param[in]  has_end_deriv2   indicates whether end derivative D2 is specified.
  //! \param[in]  D0               derivative D1 at the first point.
  //! \param[in]  Dn               derivative D1 at the last point.
  //! \param[in]  D20              derivative D2 at the first point.
  //! \param[in]  D2n              derivative D2 at the last point.
  //! \param[out] crv              result.
  //! \return true in case of success, false -- otherwise.
  mobiusGeom_EXPORT static bool
    Interp(const std::vector<t_xyz>& points,
           const int                 n,
           const int                 p,
           const double*             params,
           const double*             pU,
           const int                 m,
           const bool                has_start_deriv,
           const bool                has_end_deriv,
           const bool                has_start_deriv2,
           const bool                has_end_deriv2,
           const t_xyz&              D0,
           const t_xyz&              Dn,
           const t_xyz&              D20,
           const t_xyz&              D2n,
           t_ptr<t_bcurve>&          crv);

public:

  //! Accessor for error code.
  //! \return error code.
  int GetErrorCode() const
  {
    return m_errCode;
  }

  //! Accessor for the resulting curve.
  //! \return interpolant curve.
  const t_ptr<t_bcurve>& GetResult() const
  {
    return m_curve;
  }

protected:

  //! Returns index of the last pole. Notice that this index is zero-based,
  //! so if we have K poles, it will return (K-1).
  //! \return index of the last pole.
  int last_index_poles() const;

  //! Returns index of the last knot. Notice that this index is zero-based,
  //! so if we have K knots, it will return (K-1).
  //! \return index of the last knot.
  int last_index_knots() const;

  //! Returns true if start derivative D1 is specified, false -- otherwise.
  //! \return true/false.
  bool has_start_deriv() const;

  //! Returns true if end derivative D1 is specified, false -- otherwise.
  //! \return true/false.
  bool has_end_deriv() const;

  //! Returns true if start derivative D2 is specified, false -- otherwise.
  //! \return true/false.
  bool has_start_deriv2() const;

  //! Returns true if end derivative D2 is specified, false -- otherwise.
  //! \return true/false.
  bool has_end_deriv2() const;

protected:

  //! Returns dimension of the problem: number of unknown variables and
  //! equations in the linear system to solve.
  //! \param[in] n                last pole index.
  //! \param[in] has_start_deriv  indicates whether start derivative D1 is specified.
  //! \param[in] has_end_deriv    indicates whether end derivative D1 is specified.
  //! \param[in] has_start_deriv2 indicates whether start derivative D2 is specified.
  //! \param[in] has_end_deriv2   indicates whether end derivative D2 is specified.
  //! \return dimension.
  static int dimension(const int  n,
                       const bool has_start_deriv,
                       const bool has_end_deriv,
                       const bool has_start_deriv2,
                       const bool has_end_deriv2);

private:

  int                  m_iDeg;       //!< Degree of interpolant curve.
  ErrCode              m_errCode;    //!< Error code.
  std::vector<t_xyz>   m_points;     //!< Points to interpolate.
  t_xyz                m_D0;         //!< Derivative D1 at the first point.
  t_xyz                m_Dn;         //!< Derivative D1 at the last point.
  t_xyz                m_D20;        //!< Derivative D2 at the first point.
  t_xyz                m_D2n;        //!< Derivative D2 at the last point.
  bspl_ParamsSelection m_paramsType; //!< Parameterization type.
  double*              m_pParams;    //!< Parameters externally defined for interpolation.
  int                  m_iNumParams; //!< Number of parameters.
  bspl_KnotsSelection  m_knotsType;  //!< Knots selection type.
  double*              m_pU;         //!< Knot vector externally defined for interpolation.
  int                  m_iNumKnots;  //!< Number of knots.
  t_ptr<t_bcurve>      m_curve;      //!< Interpolant curve.

};

};

#endif
