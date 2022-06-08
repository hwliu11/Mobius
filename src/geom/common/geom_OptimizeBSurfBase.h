//-----------------------------------------------------------------------------
// Created on: 21 June 2019
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

#ifndef geom_OptimizeBSurfBase_HeaderFile
#define geom_OptimizeBSurfBase_HeaderFile

// Geometry includes
#include <mobius/geom_BSplineSurface.h>
#include <mobius/geom_BSurfNk.h>

// Core includes
#include <mobius/core_OPERATOR.h>

// Standard includes
#include <set>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Base class for surface optimization operators (fairing, approximation, etc.).
class geom_OptimizeBSurfBase : public core_OPERATOR
{
public:

  //! Ctor.
  //! \param[in] progress progress notifier.
  //! \param[in] plotter  imperative plotter.
  geom_OptimizeBSurfBase(core_ProgressEntry progress,
                         core_PlotterEntry  plotter)
  : core_OPERATOR(progress, plotter)
  {}

  //! Ctor accepting initial surface.
  //! \param[in] initSurf initial surface.
  //! \param[in] progress progress notifier.
  //! \param[in] plotter  imperative plotter.
  geom_OptimizeBSurfBase(const t_ptr<t_bsurf>& initSurf,
                         core_ProgressEntry    progress,
                         core_PlotterEntry     plotter)
  : core_OPERATOR(progress, plotter)
  {
    this->SetInitSurface(initSurf);
  }

public:

  //! Sets initial surface. The initial surface is a B-surface which principally
  //! follows the shape of a point cloud, i.e., each point can be inverted to
  //! that surface unambiguously.
  //! \param[in] initSurf initial surface to set.
  void SetInitSurface(const t_ptr<t_bsurf>& initSurf)
  {
    m_initSurf   = initSurf;
    m_iNumPolesU = int( m_initSurf->GetPoles().size() );
    m_iNumPolesV = int( m_initSurf->GetPoles()[0].size() );
  }

  //! \return resulting surface.
  const t_ptr<t_bsurf>& GetResult() const
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

protected:

  //! Constructs basis functions in advance.
  //! \param[in] alloc memory arena.
  //! \return true in case of success, false -- otherwise.
  bool prepareNk(t_ptr<t_alloc2d> alloc)
  {
    // Contract check.
    if ( m_iNumPolesU <= 0 || m_iNumPolesV <= 0 )
    {
      m_progress.SendLogMessage(MobiusErr(Normal) << "Numbers of U and V poles are incorrect.");
      return false;
    }

    const int nPoles = m_iNumPolesU*m_iNumPolesV;
    for ( int k = 0; k < nPoles; ++k )
    {
      // Prepare evaluator for N_k(u,v).
      t_ptr<geom_BSurfNk>
        Nk = new geom_BSurfNk(m_initSurf, k, alloc);
      //
      m_Nk.push_back(Nk);
    }

    return true; // Basis functions are all ready.
  }

protected:

  //! Surface to optimize.
  t_ptr<t_bsurf> m_initSurf;

  //! Result of optimization.
  t_ptr<t_bsurf> m_resultSurf;

  //! Number of poles in U direction.
  int m_iNumPolesU;

  //! Number of poles in V direction (used to compute serial indices of poles).
  int m_iNumPolesV;

  //! Serial indices of control points to pin.
  std::set<int> m_pinnedPoles;

  //! Evaluators of \f$N_k(u,v)\f$ functions.
  std::vector< t_ptr<geom_BSurfNk> > m_Nk;

};

}

#endif
