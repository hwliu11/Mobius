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

#ifndef geom_BuildAveragePlane_h
#define geom_BuildAveragePlane_h

// Geom includes
#include <mobius/geom_PlaneSurface.h>
#include <mobius/geom_PositionCloud.h>

// Core includes
#include <mobius/core_OPERATOR.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Utility to build an average plane for the given point set.
class geom_BuildAveragePlane : public core_OPERATOR
{
public:

  //! Constructs the tool.
  //! \param[in] progress progress indicator.
  //! \param[in] plotter  imperative plotter.
  mobiusGeom_EXPORT
    geom_BuildAveragePlane(core_ProgressEntry progress = NULL,
                           core_PlotterEntry  plotter  = NULL);

public:

  //! Constructs the average plane on the given point set passed in
  //! the form of a point cloud.
  //! \param[in]  points point set to build a fitting plane for.
  //! \param[out] result resulting plane.
  //! \return true in case of success, false -- otherwise.
  mobiusGeom_EXPORT bool
    Build(const t_ptr<t_pcloud>& points,
          t_ptr<t_plane>&        result) const;

};

};

#endif
