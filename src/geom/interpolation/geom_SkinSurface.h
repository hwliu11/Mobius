//-----------------------------------------------------------------------------
// Created on: 09 March 2015
//-----------------------------------------------------------------------------
// Copyright (c) 2015-present, Sergey Slyadnev
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

// Geom includes
#include <mobius/geom_BSplineCurve.h>
#include <mobius/geom_BSplineSurface.h>

// Core includes
#include <mobius/core_HeapAlloc.h>
#include <mobius/core_OPERATOR.h>

// BSpl includes
#include <mobius/bspl_KnotsSelection.h>
#include <mobius/bspl_ParamsSelection.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Creates B-surface passing through the given series of B-curves. The
//! constructed surface will have the input curves as its `u = const`
//! isolines. The skinning process follows the algorithm described in
//! paragraph 10.3 of The NURBS Books
//!
//! [Piegl, L. and Tiller, W. 1995. The NURBS Book. Springer Berlin Heidelberg, Berlin, Heidelberg].
//!
//! At the first stage, the input curves are made compatible (this stage can
//! be skipped by a dedicated flag). Then, the control points of the
//! input curves are interpolated and the poles of the interpolant curves
//! are taken as the resulting poles for a B-surface.
class geom_SkinSurface : public core_OPERATOR
{
public:

  //! Error codes.
  enum ErrCode
  {
    ErrCode_NoError = 0,
    ErrCode_NotInitialized,
    ErrCode_NotDone,
    ErrCode_NotEnoughCurves,
    ErrCode_NotCompatibleCurves,
    ErrCode_NotDoneUnification,
    ErrCode_BadVDegree,
    ErrCode_CannotSelectParameters,
    ErrCode_CannotSelectKnots,
    ErrCode_CannotInterpolateIsoU
  };

public:

  //! Default ctor.
  //! \param[in] progress progress notifier.
  //! \param[in] plotter  imperative plotter.
  mobiusGeom_EXPORT
    geom_SkinSurface(core_ProgressEntry progress = nullptr,
                     core_PlotterEntry  plotter  = nullptr);

  //! Complete constructor. Initializes the interpolation tool with a set
  //! of curves to skin the surface through.
  //! \param[in] curves      curves to skin the surface through.
  //! \param[in] deg_V       degree in V curvilinear direction.
  //! \param[in] unifyCurves indicates whether to unify curves before skinning.
  //! \param[in] progress    progress notifier.
  //! \param[in] plotter     imperative plotter.
  mobiusGeom_EXPORT
    geom_SkinSurface(const std::vector< t_ptr<t_bcurve> >& curves,
                     const int                             deg_V,
                     const bool                            unifyCurves,
                     core_ProgressEntry                    progress = nullptr,
                     core_PlotterEntry                     plotter  = nullptr);

public:

  //! Initializes skinning tool.
  //! \param[in] curves      curves to skin the surface through.
  //! \param[in] deg_V       degree in V curvilinear direction.
  //! \param[in] unifyCurves indicates whether to unify curves before skinning.
  mobiusGeom_EXPORT void
    Init(const std::vector< t_ptr<t_bcurve> >& curves,
         const int                             deg_V,
         const bool                            unifyCurves);

  //! Sets tangency constraints for the poles of the leading section. The
  //! passed array will be treated as an array of vectors where index of an
  //! element corresponds to the index of a certain iso-U interpolant.
  //! \param[in] tangencies tangency constraints to set.
  mobiusGeom_EXPORT void
    AddLeadingTangencies(const std::vector<t_xyz>& tangencies);

  //! Sets tangency constraints for the poles of the trailing section. The
  //! passed array will be treated as an array of vectors where index of an
  //! element corresponds to the index of a certain iso-U interpolant.
  //! \param[in] tangencies tangency constraints to set.
  mobiusGeom_EXPORT void
    AddTrailingTangencies(const std::vector<t_xyz>& tangencies);

  //! Performs skinning.
  //! \return true in case of success, false -- otherwise.
  mobiusGeom_EXPORT bool
    Perform();

public:

  //! Prepares input section curves for skinning. This method first checks
  //! if the curves are compatible. If not, and if unification is allowed
  //! by `unifyCurves` flag passed at initialization, the curves are made
  //! compatible.
  //! \return true if the sections are ready for skinning, false -- otherwise.
  mobiusGeom_EXPORT bool
    PrepareSections();

  //! Builds intermediate curves by interpolating the poles of the section
  //! curves. This method is essentially the skinning itself. Once this method
  //! is done, the only remaining thing is to organize the obtained control
  //! points in a grid and construct the resulting B-surface explicitly.
  //! \return true in case of success, false -- otherwise.
  mobiusGeom_EXPORT bool
    BuildIsosU();

  //! Constructs the resulting B-surface.
  //! \return true in case of success, false -- otherwise.
  mobiusGeom_EXPORT bool
    BuildSurface();

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

  //! Forces the algorithm to use the passed parameters instead of recomputing them.
  //! \param[in] params the parameters to set.
  void ForceParameters(const std::vector<double>& params)
  {
    m_params = params;
  }

  //! Forces the algorithm to use the passed knots instead of recomputing them.
  //! \param[in] params the parameters to set.
  void ForceKnots(const std::vector<double>& knots)
  {
    m_V = knots;
  }

  //! Specifies whether to use chord-length parameterization along V. Alternatively,
  //! it is the centripetal parameterization.
  void SetUseChordLength(const bool on)
  {
    m_bChord = on;
  }

  //! \return the computed knots.
  const std::vector<double>& GetResultKnots() const
  {
    return m_V;
  }

  //! \return the computed parameters.
  const std::vector<double>& GetResultParams() const
  {
    return m_params;
  }

public:

  //! Intermediate interpolants.
  std::vector< t_ptr<t_bcurve> > IsoU_Curves;

private:

  core_HeapAlloc<double>         m_alloc;   //!< Heap allocator.
  std::vector< t_ptr<t_bcurve> > m_curves;  //!< Curves to interpolate.
  std::vector<t_xyz>             m_D1lead;  //!< Leading tangency constraints.
  std::vector<t_xyz>             m_D1tail;  //!< Trailing tangency constraints.
  std::vector<double>            m_V;       //!< Knot vector in V direction.
  std::vector<double>            m_params;  //!< Computed parameters.
  int                            m_iDeg_V;  //!< V-degree of interpolant surface.
  bool                           m_bUnify;  //!< Indicates whether to unify curves.
  ErrCode                        m_errCode; //!< Error code.
  t_ptr<t_bsurf>                 m_surface; //!< Interpolant surface.
  bool                           m_bChord;  //!< Whether to use chord-length parameterization.

};

}

#endif
