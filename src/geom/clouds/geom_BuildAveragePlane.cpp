//-----------------------------------------------------------------------------
// Created on: 23 May 2019
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
//    * Neither the name of the copyright holder(s) nor the
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

// Own include
#include <mobius/geom_BuildAveragePlane.h>

// Core includes
#include <mobius/core_SolveCovarianceEigens.h>

#undef COUT_DEBUG
#if defined COUT_DEBUG
  #pragma message("===== warning: COUT_DEBUG is enabled")
#endif

//-----------------------------------------------------------------------------

mobius::geom_BuildAveragePlane::geom_BuildAveragePlane(core_ProgressEntry progress,
                                                       core_PlotterEntry  plotter)
: core_OPERATOR(progress, plotter)
{}

//-----------------------------------------------------------------------------

bool mobius::geom_BuildAveragePlane::Build(const ptr<pcloud>& points,
                                           ptr<plane>&        result) const
{
  // Compute eigen vectors of a covariance matrix for the point cloud.
  core_SolveCovarianceEigens solver;
  //
  xyz C, Dx, Dy, Dz;
  if ( !solver(points->GetPoints(), C, Dx, Dy, Dz) )
  {
    m_progress.SendLogMessage(MobiusErr(Normal) << "Failed to compute eigen vectors of a covariance matrix.");
    return false;
  }

  // Build plane.
  result = new plane(C, Dx, Dy);
  return true;
}
