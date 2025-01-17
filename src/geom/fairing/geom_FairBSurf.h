//-----------------------------------------------------------------------------
// Created on: 20 August 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2018-present, Sergey Slyadnev
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

#ifndef geom_FairBSurf_HeaderFile
#define geom_FairBSurf_HeaderFile

// Geometry includes
#include <mobius/geom_OptimizeBSurfBase.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Fairing algorithm for B-spline surfaces. See
//!
//! [M. Kallay, Constrained optimization in surface design, in: Modeling in
//!  Computer Graphics, Springer Berlin Heidelberg, 1993, pp. 85-93.]
//!
//! If pin-point constraints are specified, the corresponding equations
//! are excluded from the system (hence the dimension of the problem is
//! reduced).
class geom_FairBSurf : public geom_OptimizeBSurfBase
{
public:

  //! ctor.
  //! \param[in] surface  B-spline surface to fair.
  //! \param[in] lambda   fairing coefficient.
  //! \param[in] progress progress notifier.
  //! \param[in] plotter  imperative plotter.
  mobiusGeom_EXPORT
    geom_FairBSurf(const t_ptr<t_bsurf>& surface,
                   const double          lambda,
                   core_ProgressEntry    progress,
                   core_PlotterEntry     plotter);

public:

  //! Performs fairing.
  //! \return true in case of success, false -- otherwise.
  mobiusGeom_EXPORT bool
    Perform();

protected:

  //! Fairing coefficient.
  double m_fLambda;

};

}

#endif
