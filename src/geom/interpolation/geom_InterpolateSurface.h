//-----------------------------------------------------------------------------
// Created on: 23 December 2014
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

#ifndef geom_InterpolateSurface_HeaderFile
#define geom_InterpolateSurface_HeaderFile

// Geometry includes
#include <mobius/geom_BSplineCurve.h>
#include <mobius/geom_BSplineSurface.h>
#include <mobius/geom_VectorField.h>

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
//! Interpolates B-surface over the given collection of points. See algorithm
//! A9.4 from The NURBS Book.
class geom_InterpolateSurface : public core_OPERATOR
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
    ErrCode_CannotInterpolateIsoV,
    ErrCode_CannotInterpolateIsoU,
    ErrCode_PoorInitialGrid,
    ErrCode_NonRectangularGrid,
    ErrCode_NotEnoughVDerivs_Start_D1,
    ErrCode_NotEnoughVDerivs_End_D1,
    ErrCode_NotEnoughVDerivs_Start_D2,
    ErrCode_NotEnoughVDerivs_End_D2
  };

public:

  //! Default ctor.
  //! \param[in] progress progress notifier.
  //! \param[in] plotter  imperative plotter.
  mobiusGeom_EXPORT
    geom_InterpolateSurface(core_ProgressEntry progress = NULL,
                            core_PlotterEntry  plotter  = NULL);

  //! Complete constructor. Initializes the interpolation tool with a set
  //! of reper points and other properties which are necessary in order to
  //! reduce the generally sophisticated interpolation problem to a
  //! trivial linear algebraic system.
  //!
  //! \param[in] points     data points to interpolate.
  //! \param[in] deg_U      U-degree of the interpolant surface.
  //! \param[in] deg_V      V-degree of the interpolant surface.
  //! \param[in] paramsType strategy for choosing interpolant parameters for
  //!                       the data points.
  //! \param[in] knotsType  strategy for choosing knot vector for interpolant
  //!                       B-splines.
  //! \param[in] progress   progress notifier.
  //! \param[in] plotter    imperative plotter.
  mobiusGeom_EXPORT
    geom_InterpolateSurface(const std::vector< std::vector<t_xyz> >& points,
                            const int                                deg_U,
                            const int                                deg_V,
                            const bspl_ParamsSelection               paramsType,
                            const bspl_KnotsSelection                knotsType,
                            core_ProgressEntry                       progress = NULL,
                            core_PlotterEntry                        plotter  = NULL);

  //! Complete constructor. Initializes the interpolation tool with a set
  //! of reper points and other properties which are necessary in order to
  //! reduce the generally sophisticated interpolation problem to a
  //! trivial linear algebraic system.
  //!
  //! \param[in] points               data points to interpolate.
  //! \param[in] deg_U                U-degree of the interpolant surface.
  //! \param[in] deg_V                V-degree of the interpolant surface.
  //! \param[in] derivs_isoV_start_D1 leading V-iso derivatives D1.
  //! \param[in] derivs_isoV_end_D1   trailing V-iso derivatives D1.
  //! \param[in] paramsType           strategy for choosing interpolant parameters for
  //!                                 the data points.
  //! \param[in] knotsType            strategy for choosing knot vector for interpolant
  //!                                 B-splines.
  //! \param[in] progress             progress notifier.
  //! \param[in] plotter              imperative plotter.
  mobiusGeom_EXPORT
    geom_InterpolateSurface(const std::vector< std::vector<t_xyz> >& points,
                            const int                                deg_U,
                            const int                                deg_V,
                            const t_ptr<geom_VectorField>&           derivs_isoV_start_D1,
                            const t_ptr<geom_VectorField>&           derivs_isoV_end_D1,
                            const bspl_ParamsSelection               paramsType,
                            const bspl_KnotsSelection                knotsType,
                            core_ProgressEntry                       progress = NULL,
                            core_PlotterEntry                        plotter  = NULL);

  //! Complete constructor. Initializes the interpolation tool with a set
  //! of reper points and other properties which are necessary in order to
  //! reduce the generally sophisticated interpolation problem to a
  //! trivial linear algebraic system.
  //!
  //! \param[in] points               data points to interpolate.
  //! \param[in] deg_U                U-degree of the interpolant surface.
  //! \param[in] deg_V                V-degree of the interpolant surface.
  //! \param[in] derivs_isoV_start_D1 leading V-iso derivatives D1.
  //! \param[in] derivs_isoV_end_D1   trailing V-iso derivatives D1.
  //! \param[in] derivs_isoV_start_D2 leading V-iso derivatives D2.
  //! \param[in] derivs_isoV_end_D2   trailing V-iso derivatives D2.
  //! \param[in] paramsType           strategy for choosing interpolant parameters for
  //!                                 the data points.
  //! \param[in] knotsType            strategy for choosing knot vector for interpolant
  //!                                 B-splines.
  //! \param[in] progress             progress notifier.
  //! \param[in] plotter              imperative plotter.
  mobiusGeom_EXPORT
    geom_InterpolateSurface(const std::vector< std::vector<t_xyz> >& points,
                            const int                                deg_U,
                            const int                                deg_V,
                            const t_ptr<geom_VectorField>&           derivs_isoV_start_D1,
                            const t_ptr<geom_VectorField>&           derivs_isoV_end_D1,
                            const t_ptr<geom_VectorField>&           derivs_isoV_start_D2,
                            const t_ptr<geom_VectorField>&           derivs_isoV_end_D2,
                            const bspl_ParamsSelection               paramsType,
                            const bspl_KnotsSelection                knotsType,
                            core_ProgressEntry                       progress = NULL,
                            core_PlotterEntry                        plotter  = NULL);

public:

  //! Initializes interpolation tool.
  //! \param[in] points     data points to interpolate.
  //! \param[in] deg_U      U-degree of the interpolant surface.
  //! \param[in] deg_V      V-degree of the interpolant surface.
  //! \param[in] paramsType strategy for choosing interpolant parameters for
  //!                       the data points.
  //! \param[in] knotsType  strategy for choosing knot vector for interpolant
  //!                       B-splines.
  mobiusGeom_EXPORT void
    Init(const std::vector< std::vector<t_xyz> >& points,
         const int                                deg_U,
         const int                                deg_V,
         const bspl_ParamsSelection               paramsType,
         const bspl_KnotsSelection                knotsType);

  //! Initializes interpolation tool.
  //! \param[in] points               data points to interpolate.
  //! \param[in] deg_U                U-degree of the interpolant surface.
  //! \param[in] deg_V                V-degree of the interpolant surface.
  //! \param[in] derivs_isoV_start_D1 leading V-iso derivatives D1.
  //! \param[in] derivs_isoV_end_D1   trailing V-iso derivatives D1.
  //! \param[in] paramsType           strategy for choosing interpolant parameters for
  //!                                 the data points.
  //! \param[in] knotsType            strategy for choosing knot vector for interpolant
  //!                                 B-splines.
  mobiusGeom_EXPORT void
    Init(const std::vector< std::vector<t_xyz> >& points,
         const int                                deg_U,
         const int                                deg_V,
         const t_ptr<geom_VectorField>&           derivs_isoV_start_D1,
         const t_ptr<geom_VectorField>&           derivs_isoV_end_D1,
         const bspl_ParamsSelection               paramsType,
         const bspl_KnotsSelection                knotsType);

  //! Initializes interpolation tool.
  //! \param[in] points               data points to interpolate.
  //! \param[in] deg_U                U-degree of the interpolant surface.
  //! \param[in] deg_V                V-degree of the interpolant surface.
  //! \param[in] derivs_isoV_start_D1 leading V-iso derivatives D1.
  //! \param[in] derivs_isoV_end_D1   trailing V-iso derivatives D1.
  //! \param[in] derivs_isoV_start_D2 leading V-iso derivatives D2.
  //! \param[in] derivs_isoV_end_D2   trailing V-iso derivatives D2.
  //! \param[in] paramsType           strategy for choosing interpolant parameters for
  //!                                 the data points.
  //! \param[in] knotsType            strategy for choosing knot vector for interpolant
  //!                                 B-splines.
  mobiusGeom_EXPORT void
    Init(const std::vector< std::vector<t_xyz> >& points,
         const int                                deg_U,
         const int                                deg_V,
         const t_ptr<geom_VectorField>&           derivs_isoV_start_D1,
         const t_ptr<geom_VectorField>&           derivs_isoV_end_D1,
         const t_ptr<geom_VectorField>&           derivs_isoV_start_D2,
         const t_ptr<geom_VectorField>&           derivs_isoV_end_D2,
         const bspl_ParamsSelection               paramsType,
         const bspl_KnotsSelection                knotsType);

public:

  //! Performs interpolation.
  //! \return true in case of success, false -- otherwise.
  mobiusGeom_EXPORT bool
    Perform();

public:

  //! Accessor for error code.
  //! \return error code.
  int GetErrorCode() const
  {
    return m_errCode;
  }

  //! Accessor for the resulting surface.
  //! \return interpolant surface.
  const t_ptr<t_bsurf>& GetResult() const
  {
    return m_surface;
  }

private:

  int                               m_iDeg_U;     //!< U-degree of interpolant surface.
  int                               m_iDeg_V;     //!< V-degree of interpolant surface.
  ErrCode                           m_errCode;    //!< Error code.
  std::vector< std::vector<t_xyz> > m_points;     //!< Points to interpolate.
  bspl_ParamsSelection              m_paramsType; //!< Parameterization type.
  bspl_KnotsSelection               m_knotsType;  //!< Knots selection type.
  t_ptr<t_bsurf>                    m_surface;    //!< Interpolant surface.

// Derivative constraints:
private:

  t_ptr<geom_VectorField> m_derivs_isoV_start_D1; //!< Leading derivative D1 constraints on V iso.
  t_ptr<geom_VectorField> m_derivs_isoV_end_D1;   //!< Trailing derivative D1 constraints on V iso.

  t_ptr<geom_VectorField> m_derivs_isoV_start_D2; //!< Leading derivative D2 constraints on V iso.
  t_ptr<geom_VectorField> m_derivs_isoV_end_D2;   //!< Trailing derivative D2 constraints on V iso.

// For maintenance:
public:

  std::vector< t_ptr<t_bcurve> > IsoV_Curves;
  std::vector< t_ptr<t_bcurve> > ReperU_Curves;

};

};

#endif
