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
//! point clouds.
class geom_ApproxBSurf : public core_OPERATOR
{
public:

  //! Ctor.
  //! \param[in] points    points to approximate.
  //! \param[in] uDegree   desired U degree.
  //! \param[in] vDegree   desired V degree.
  //! \param[in] numPolesU number of poles in U direction (do not mix up with
  //!                      "fixed-U direction" which is a "V direction".
  //! \param[in] numPolesV number of poles in V direction.
  //! \param[in] progress  progress notifier.
  //! \param[in] plotter   imperative plotter.
  mobiusGeom_EXPORT
    geom_ApproxBSurf(const t_ptr<t_pcloud>& points,
                     const int              uDegree,
                     const int              vDegree,
                     const int              numPolesU,
                     const int              numPolesV,
                     core_ProgressEntry     progress,
                     core_PlotterEntry      plotter);

public:

  //! Performs approximation.
  //! \return true in case of success, false -- otherwise.
  mobiusGeom_EXPORT bool
    Perform();

public:

  //! \return resulting surface.
  const t_ptr<t_bsurf>& GetResult() const
  {
    return m_resultSurf;
  }

private:

  void prepareNk(t_ptr<t_alloc2d> alloc);

protected:

  //! Points to approximate.
  t_ptr<t_pcloud> m_inputPoints;

  //! Approximated surface.
  t_ptr<t_bsurf> m_resultSurf;

  //! U degree.
  int m_iDegreeU;

  //! V degree.
  int m_iDegreeV;

  //! Number of poles in U direction.
  int m_iNumPolesU;

  //! Number of poles in V direction.
  int m_iNumPolesV;

  //! U knot vector.
  std::vector<double> m_U;

  //! V knot vector.
  std::vector<double> m_V;

  //! Evaluators of \f$N_k(u,v)\f$ functions.
  std::vector< t_ptr<geom_BSurfNk> > m_Nk;

};

};

#endif
