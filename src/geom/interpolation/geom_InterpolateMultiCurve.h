//-----------------------------------------------------------------------------
// Created on: 03 December 2018
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

#ifndef geom_InterpolateMultiCurve_HeaderFile
#define geom_InterpolateMultiCurve_HeaderFile

// Geometry includes
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
//! Interpolates B-curves over the given collections of points. This utility
//! ensures compatibility of interpolant curves for subsequent surfacing
//! operators (e.g. skinning).
class geom_InterpolateMultiCurve : public core_OPERATOR
{
public:

  //! Ctor accepting common interpolation parameters.
  //! \param[in] deg        degree of interpolant curves.
  //! \param[in] paramsType type of parameterization.
  //! \param[in] knotsType  strategy of knots selection.
  //! \param[in] progress   progress notifier.
  //! \param[in] plotter    imperative plotter.
  mobiusGeom_EXPORT
    geom_InterpolateMultiCurve(const int                  deg,
                               const bspl_ParamsSelection paramsType,
                               const bspl_KnotsSelection  knotsType,
                               core_ProgressEntry         progress = nullptr,
                               core_PlotterEntry          plotter  = nullptr);

public:

  //! Adds a row of points to interpolate.
  //! \param[in] points ordered points to interpolate.
  mobiusGeom_EXPORT void
    AddRow(const std::vector<t_xyz>& points);

  //! Performs interpolation.
  //! \return true in case of success, false -- otherwise.
  mobiusGeom_EXPORT bool
    Perform();

public:

  //! Accessor for the resulting curve.
  //! \param[in] idx 0-based index of the curve to access.
  //! \return interpolant curve.
  const t_ptr<t_bcurve>& GetResult(const int idx) const
  {
    return m_curves[idx];
  }

  //! \return number of multi-point constraints.
  int GetNumRows() const
  {
    return int( m_pointsGrid.size() );
  }

private:

  int                               m_iDeg;       //!< Degree of interpolant curve.
  std::vector< std::vector<t_xyz> > m_pointsGrid; //!< Points to interpolate.
  bspl_ParamsSelection              m_paramsType; //!< Parameterization type.
  bspl_KnotsSelection               m_knotsType;  //!< Knots selection type.
  std::vector< t_ptr<t_bcurve> >    m_curves;     //!< Interpolant curves.

};

};

#endif
