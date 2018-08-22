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

#ifndef geom_FairBSurfNk_HeaderFile
#define geom_FairBSurfNk_HeaderFile

// Geom includes
#include <mobius/geom_BSplineSurface.h>

// BSpl includes
#include <mobius/bspl.h>

// Core includes
#include <mobius/core_HeapAlloc.h>
#include <mobius/core_UV.h>

// Standard includes
#include <unordered_map>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Function representing a product \f$N_k(u,v)\f$ of two B-spline basis
//! functions \f$N_i^p(u)\f$ and \f$N_j^q(v)\f$ with their derivatives.
//!
//! \f[ \textbf{s}(u,v) = \sum_i \sum_j \textbf{P}_{ij} N_i^p(u) N_j^q(v) = \sum_k \textbf{C}_k N_k(u,v) \f]
//!
//! To use functions \f$N_k(u,v)\f$, all control points of the surface \f$\textbf{s}(u,v)\f$
//! have to be renumbered.
class geom_FairBSurfNk : public core_OBJECT
{
public:

  //! \brief Ctor accepting all properties of the involved i-th and j-th
  //!        basis functions.
  //! \param[in] surface B-spline surface to take basis functions from.
  //! \param[in] k       0-based serial index.
  //! \param[in] alloc   shared allocator.
  geom_FairBSurfNk(const ptr<bsurf>& surface,
                   const int         k,
                   ptr<alloc2d>      alloc)
  : core_OBJECT (),
    m_alloc     (alloc)
  {
    // Initialize Ni and Nj from surface.
    m_Ni.U = surface->Knots_U();
    m_Ni.p = surface->Degree_U();
    //
    m_Nj.V = surface->Knots_V();
    m_Nj.q = surface->Degree_V();
    //
    bspl::PairIndicesFromSerial(k, int( surface->Poles()[0].size() ), m_Ni.i, m_Nj.j);
  }

public:

  //! Evaluates function up to its second derivatives.
  //! \param[in]  u       first parameter value.
  //! \param[in]  v       second parameter value.
  //! \param[out] N       primal value.
  //! \param[out] dN_dU   first-order derivative by U.
  //! \param[out] dN_dV   first-order derivative by V.
  //! \param[out] d2N_dU2 second-order derivative by U.
  //! \param[out] d2N_dUV second-order derivative by U and V.
  //! \param[out] d2N_dV2 second-order derivative by V.
  mobiusGeom_EXPORT virtual void
    Eval_D2(const double u,
            const double v,
            double&      N,
            double&      dN_dU,
            double&      dN_dV,
            double&      d2N_dU2,
            double&      d2N_dUV,
            double&      d2N_dV2);

public:

  //! Returns the indices of knot spans whether the function \f$N_k(u,v)\f$
  //! is not zero. See for example P2.1 at p. 55 in "The NURBS Book".
  void GetSupportSpans(int& ifirst, int& ilast, int& jfirst, int& jlast)
  {
    ifirst = m_Ni.i;
    ilast  = m_Ni.i + m_Ni.p + 1;
    jfirst = m_Nj.j;
    jlast  = m_Nj.j + m_Nj.q + 1;
  }

protected:

  struct t_cell
  {
    uv  coords;
    int indices[2];

    t_cell() { indices[0] = indices[1] = 0; } //!< Default ctor.

    //! Complete ctor.
    t_cell(const uv& p, const double cellSize)
    {
      for ( int k = 0; k < 2; ++k )
      {
        double val = p.Coord(k) / cellSize;

        // If the value of index is greater than INT_MAX, it is decreased
        // correspondingly for the value of INT_MAX. If the value of index is
        // less than INT_MIN, it is increased correspondingly for the absolute
        // value of INT_MIN.
        indices[k] = long((val > INT_MAX - 1) ? fmod(val, (double) INT_MAX)  : (val < INT_MIN + 1) ? fmod(val, (double) INT_MIN) : val);
      }
    }

    bool operator==(const t_cell& cell) const
    {
      return (this->indices[0] == cell.indices[0]) && (this->indices[1] == cell.indices[1]);
    }

    struct hasher
    {
      size_t operator()(const t_cell& cell) const
      {
        return cell.indices[0] ^ cell.indices[1];
      }
    };
  };

  struct t_values
  {
    double N;
    double dN_dU;
    double dN_dV;
    double d2N_dU2;
    double d2N_dUV;
    double d2N_dV2;

    t_values() : N(0.), dN_dU(0.), dN_dV(0.), d2N_dU2(0.), d2N_dUV(0.), d2N_dV2(0.) {}
  };

protected:

  //! Cached values of \f$N_k(u,v)\f$ for certain pairs of \f$(u,v)\f$
  //! parameters. Such caching technique is effective because the functions
  //! \f$N_k(u,v)\f$ are evaluated always in the same predefined points used
  //! by Gauss integration scheme.
  std::unordered_map<t_cell, t_values, t_cell::hasher> m_cells;

  ptr<alloc2d> m_alloc; //!< Shared memory allocator.

  //! First basis function \f$N_i^p(u)\f$.
  struct
  {
    std::vector<double> U; //!< Knot vector.
    int                 p; //!< Degree.
    int                 i; //!< 0-based index.
  } m_Ni;

  //! Second basis function \f$N_j^q(v)\f$.
  struct
  {
    std::vector<double> V; //!< Knot vector.
    int                 q; //!< Degree.
    int                 j; //!< 0-based index.
  } m_Nj;

};

};

#endif
