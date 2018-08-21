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
#include <mobius/geom_BSplineSurface.h>
#include <mobius/geom_FairBSurfNk.h>

// BSpl includes
#include <mobius/bspl.h>

// Core includes
#include <mobius/core_OPERATOR.h>

// Standard includes
#include <set>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Fairing algorithm for B-spline surfaces. See
//!
//! [M. Kallay, Constrained optimization in surface design, in: Modeling in
//!  Computer Graphics, Springer Berlin Heidelberg, 1993, pp. 85-93.]
class geom_FairBSurf : public core_OPERATOR
{
public:

  //! ctor.
  //! \param[in] surface  B-spline surface to fair.
  //! \param[in] lambda   fairing coefficient.
  //! \param[in] progress progress notifier.
  //! \param[in] plotter  imperative plotter.
  mobiusGeom_EXPORT
    geom_FairBSurf(const ptr<bsurf>&  surface,
                   const double       lambda,
                   core_ProgressEntry progress,
                   core_PlotterEntry  plotter);

public:

  //! Performs fairing.
  //! \return true in case of success, false -- otherwise.
  mobiusGeom_EXPORT bool
    Perform();

public:

  //! \return resulting surface.
  const ptr<bsurf>& GetResult() const
  {
    return m_resultSurf;
  }

  //! Converts (i,j) indices to serial index.
  //! \param[in] i 0-based index of row.
  //! \param[in] j 0-based index of column.
  //! \return 0-based serial index of (i,j)-th element.
  int GetK(const int i, const int j) const
  {
    return bspl::SerialIndexFromPair(i, j, m_iNumPolesV);
  }

  //! Converts serial index of an element to its grid indices (i,j).
  //! \param[in]  k 0-based serial index of element.
  //! \param[out] i 0-based index of the corresponding row.
  //! \param[out] j 0-based index of the corresponding column.
  void GetIJ(const int k, int& i, int& j) const
  {
    bspl::PairIndicesFromSerial(k, m_iNumPolesV, i, j);
  }

  //! Checks whether the pole with the passed serial index is pinned or not.
  //! \param[in] k 0-based serial index to check.
  //! \return true/false.
  bool IsPinned(const int k)
  {
    return m_pinnedPoles.find(k) != m_pinnedPoles.end();
  }

  //! Checks whether the pole with the passed (i,j) indices is pinned or not.
  //! \param[in] i 0-based index of row.
  //! \param[in] j 0-based index of column.
  //! \return true/false.
  bool IsPinned(const int i, const int j)
  {
    return this->IsPinned( this->GetK(i, j) );
  }

  //! Add index of pinned pole.
  //! \param[in] i 0-based index of row.
  //! \param[in] j 0-based index of column.
  void AddPinnedPole(const int i, const int j)
  {
    m_pinnedPoles.insert( this->GetK(i, j) );
  }

  //! \return number of pinned poles.
  int GetNumPinnedPoles()
  {
    return int( m_pinnedPoles.size() );
  }

private:

  void prepareNk(ptr<alloc2d> alloc);

protected:

  //! Surface to fair.
  ptr<bsurf> m_inputSurf;

  //! Result of fairing.
  ptr<bsurf> m_resultSurf;

  //! Fairing coefficient.
  double m_fLambda;

  //! Number of poles in U direction.
  int m_iNumPolesU;

  //! Number of poles in V direction (used to compute serial indices of poles).
  int m_iNumPolesV;

  //! Serial indices of control points to pin.
  std::set<int> m_pinnedPoles;

  //! Evaluators of \f$N_k(u,v)\f$ functions.
  std::vector< ptr<geom_FairBSurfNk> > m_Nk;

};

};

#endif
