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
#include <mobius/geom_BSplineSurface.h>
#include <mobius/geom_BSurfNk.h>
#include <mobius/geom_PositionCloud.h>

// Core includes
#include <mobius/core_OPERATOR.h>

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
class geom_ApproxBSurf : public core_OPERATOR
{
public:

  //! Ctor accepting the initial B-surface.
  //! \param[in] points   points to approximate.
  //! \param[in] progress progress notifier.
  //! \param[in] plotter  imperative plotter.
  mobiusGeom_EXPORT
    geom_ApproxBSurf(const t_ptr<t_pcloud>& points,
                     const t_ptr<t_bsurf>&  initSurf,
                     core_ProgressEntry     progress = NULL,
                     core_PlotterEntry      plotter  = NULL);

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
                     core_ProgressEntry     progress = NULL,
                     core_PlotterEntry      plotter  = NULL);

public:

  //! Sets initial surface. The initial surface is a B-surface which principally
  //! follows the shape of a point cloud, i.e., each point can be inverted to
  //! that surface unambiguously.
  //! \param[in] initSurf initial surface to set.
  mobiusGeom_EXPORT void
    SetInitSurface(const t_ptr<t_bsurf>& initSurf);

  //! Performs approximation.
  //! \param[in] lambda fairing coefficient.
  //! \return true in case of success, false -- otherwise.
  mobiusGeom_EXPORT bool
    Perform(const double lambda);

public:

  //! Converts serial index of an element to its grid indices (i,j).
  //! \param[in]  k 0-based serial index of element.
  //! \param[out] i 0-based index of the corresponding row.
  //! \param[out] j 0-based index of the corresponding column.
  void GetIJ(const int k, int& i, int& j) const
  {
    bspl::PairIndicesFromSerial(k, m_initSurf->GetNumOfPoles_V(), i, j);
  }

  //! \return resulting surface.
  const t_ptr<t_bsurf>& GetResult() const
  {
    return m_resultSurf;
  }

private:

  void prepareNk(t_ptr<t_alloc2d> alloc);

  bool initializeSurf();

protected:

  //! Points to approximate.
  t_ptr<t_pcloud> m_inputPoints;

  //! Initial surface for point cloud parameterization.
  t_ptr<t_bsurf> m_initSurf;

  //! U degree.
  int m_iDegreeU;

  //! V degree.
  int m_iDegreeV;

  //! Evaluators of \f$N_k(u,v)\f$ functions.
  std::vector< t_ptr<geom_BSurfNk> > m_Nk;

  //! Parameterization of input points.
  std::vector<t_uv> m_UVs;

  //! Approximated surface.
  t_ptr<t_bsurf> m_resultSurf;

};

};

#endif
