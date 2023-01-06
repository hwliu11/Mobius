//-----------------------------------------------------------------------------
// Created on: 05 January 2023
//-----------------------------------------------------------------------------
// Copyright (c) 2013-present, Sergey Slyadnev
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

#ifndef geom_ReadAstra_HeaderFile
#define geom_ReadAstra_HeaderFile

// Geom includes
#include <mobius/geom_BSplineCurve.h>
#include <mobius/geom_BSplineSurface.h>

// Core includes
#include <mobius/core_IAlgorithm.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Utility to read ASTRA curves and surfaces.
class geom_ReadAstra : public core_IAlgorithm
{
public:

  //! Ctor.
  //! \param[in] progress progress notifier.
  //! \param[in] plotter  imperative plotter.
  geom_ReadAstra(core_ProgressEntry progress = nullptr,
                 core_PlotterEntry  plotter  = nullptr)
   : core_IAlgorithm(progress, plotter)
  {}

public:

  //! \return the imported curves.
  const std::vector< t_ptr<t_bcurve> >& GetResultCurves() const
  {
    return m_curves;
  }

  //! \return the imported surface.
  const std::vector< t_ptr<t_bsurf> >& GetResultSurfaces() const
  {
    return m_surfaces;
  }

public:

  //! Reads ASTRA file.
  //! \param[in] filename file to read.
  //! \return true in the case of success, false -- otherwise.
  mobiusGeom_EXPORT bool
    Perform(const std::string& filename);

protected:

  std::vector< t_ptr<t_bcurve> > m_curves;   //!< The imported b-spline curves.
  std::vector< t_ptr<t_bsurf> >  m_surfaces; //!< The imported b-spline surfaces.

};

}

#endif
