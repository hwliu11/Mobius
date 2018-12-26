//-----------------------------------------------------------------------------
// Created on: 26 December 2018
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

#ifndef geom_UnifyBCurves_HeaderFile
#define geom_UnifyBCurves_HeaderFile

// Geometry includes
#include <mobius/geom_BSplineCurve.h>

// Core includes
#include <mobius/core_OPERATOR.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Operator to unify B-curves so that to make them ready for interpolation
//! (e.g., by skinning or by transfinite schemes like building Coons patches).
class geom_UnifyBCurves : public core_OPERATOR
{
public:

  //! Ctor.
  //! \param[in] progress progress notifier.
  //! \param[in] plotter  imperative plotter.
  geom_UnifyBCurves(core_ProgressEntry progress,
                    core_PlotterEntry  plotter) : core_OPERATOR(progress, plotter) {}

public:

  //! Adds another curve to unify.
  //! \param[in] curve another curve to add.
  void AddCurve(const ptr<bcurve> curve)
  {
    m_curves.push_back(curve);
  }

public:

  //! Checks whether the input curves are compatible or not. This method
  //! takes degree and the knot vector of the first curve and checks if all
  //! other curves have exactly the same properties as the first one.
  //! \return true if all curves are compatible, false -- otherwise.
  mobiusGeom_EXPORT bool
    AreCompatible() const;

  //! Performs unification.
  //! \return true in case of success, false -- otherwise.
  mobiusGeom_EXPORT bool
    Perform();

public:

  //! \return resulting collection of compatible curves.
  const std::vector< ptr<bcurve> >& GetResult() const
  {
    return m_curves;
  }

protected:

  std::vector< ptr<bcurve> > m_curves; //!< Curves to make compatible.

};

};

#endif
