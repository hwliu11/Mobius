//-----------------------------------------------------------------------------
// Created on: 16 June 2019
//-----------------------------------------------------------------------------
// Copyright (c) 2019-present, Sergey Slyadnev
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

#ifndef geom_ApproxBSurf_HeaderFile
#define geom_ApproxBSurf_HeaderFile

// Geometry includes
#include <mobius/geom_OptimizeBSurfBase.h>
#include <mobius/geom_PositionCloud.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Approximation algorithm for B-spline surfaces from unstructured
//! point clouds. This algorithm implements a well-known approach to
//! surface fitting based on minimization of the following aggregated
//! functional:
//!
//! \f[ F = \sum_k{\left( \textbf{s}(u,v) - \textbf{R}_k \right)^2} + \lambda \int \int \left( \textbf{s}_{uu}^2 + 2 \textbf{s}_{uv}^2 + \textbf{s}_{vv}^2 \right) du dv \f]
//!
//! You may find more details in the paper
//!
//! [Weiss, V., Andor, L., Renner, G., and Varady, T. 2002. Advanced surface fitting techniques. Computer Aided Geometric Design 19, 19-42.]
//!
//! The algorithm allows for specification of pin-point constraints. These
//! constraints do not change anything in the algorithm (the dimension of
//! the matrix is not reduced), but the final surface is constructed with
//! simply the same poles as were pinned in the initial surface (i.e., the
//! new coordinates computed by the algorithm are simply ignored). It is
//! necessary to include the pinned points to the matrix of linear equations
//! as otherwise the final surface may expose significant oscillations because
//! the neighborhood of the pinned points remains unconstrained (e.g.,
//! parameterization of the surface can happen to be very fast near the
//! pinned corners, and the result will become oscillatory).
class geom_ApproxBSurf : public geom_OptimizeBSurfBase
{
public:

  //! Ctor accepting the initial B-surface.
  //! \param[in] points   points to approximate.
  //! \param[in] progress progress notifier.
  //! \param[in] plotter  imperative plotter.
  mobiusGeom_EXPORT
    geom_ApproxBSurf(const t_ptr<t_pcloud>& points,
                     const t_ptr<t_bsurf>&  initSurf,
                     core_ProgressEntry     progress = nullptr,
                     core_PlotterEntry      plotter  = nullptr);

  //! Ctor accepting the desired degrees.
  //! \param[in] points   points to approximate.
  //! \param[in] uDegree  U degree.
  //! \param[in] vDegree  V degree.
  //! \param[in] progress progress notifier.
  //! \param[in] plotter  imperative plotter.
  mobiusGeom_EXPORT
    geom_ApproxBSurf(const t_ptr<t_pcloud>& points,
                     const int              uDegree,
                     const int              vDegree,
                     core_ProgressEntry     progress = nullptr,
                     core_PlotterEntry      plotter  = nullptr);

public:

  //! Performs approximation.
  //! \param[in] lambda fairing coefficient.
  //! \return true in case of success, false -- otherwise.
  mobiusGeom_EXPORT bool
    Perform(const double lambda);

private:

  bool initializeSurf();

protected:

  //! Points to approximate.
  t_ptr<t_pcloud> m_inputPoints;

  //! Parameterization of input points.
  std::vector<t_uv> m_UVs;

  //! U degree.
  int m_iDegreeU;

  //! V degree.
  int m_iDegreeV;

};

}

#endif
