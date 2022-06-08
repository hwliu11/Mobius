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

#ifndef geom_BSurfNk_HeaderFile
#define geom_BSurfNk_HeaderFile

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
//! have to be renumbered. It is assumed that the controls points are
//! enumerated in V direction as rows and in U directions as columns
//! (check documentation for details).
//!
//! This function employs caching technique internally. I.e., whenever you
//! invoke its evaluation method for a certain pair of `(u,v)` coordinates,
//! the function checks whether such coordinates were evaluated before or
//! not. If yes, the cached values are returned. This caching technique
//! only requires that you use the same instance of geom_BSurfNk class as
//! long as your algorithm unfolds.
class geom_BSurfNk : public core_OBJECT
{
public:

  //! \brief Ctor accepting all properties of the involved i-th and j-th
  //!        basis functions.
  //! \param[in] surface B-spline surface to take basis functions from.
  //! \param[in] k       0-based serial index.
  //! \param[in] alloc   shared allocator.
  geom_BSurfNk(const t_ptr<t_bsurf>& surface,
               const int             k,
               t_ptr<t_alloc2d>      alloc)
  //
  : core_OBJECT (),
    m_alloc     (alloc)
  {
    // Initialize Ni and Nj from surface.
    this->init( surface->GetKnots_U(),
                surface->GetDegree_U(),
                surface->GetKnots_V(),
                surface->GetDegree_V(),
                k,
                int( surface->GetPoles()[0].size() ) - 1 );
  }

  //! \brief Ctor accepting all properties of the involved i-th and j-th
  //!        basis functions.
  //! \param[in] U     knot vector in U direction.
  //! \param[in] p     spline degree in U direction.
  //! \param[in] V     knot vector in V direction.
  //! \param[in] q     spline degree in V direction.
  //! \param[in] k     serial index of the basis Nk coeff.
  //! \param[in] n     index of the last pole in V direction.
  //! \param[in] alloc shared allocator.
  geom_BSurfNk(const std::vector<double>& U,
               const int                  p,
               const std::vector<double>& V,
               const int                  q,
               const int                  k,
               const int                  n,
               t_ptr<t_alloc2d>           alloc)
  //
  : core_OBJECT (),
    m_alloc     (alloc)
  {
    this->init(U, p, V, q, k, n);
  }

public:

  //! Evaluates function without derivatives.
  //! \param[in]  u first parameter value.
  //! \param[in]  v second parameter value.
  //! \param[out] N computed function value.
  mobiusGeom_EXPORT virtual void
    Eval(const double u,
         const double v,
         double&      N);

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
    t_uv coords;
    int  indices[2];

    t_cell() { indices[0] = indices[1] = 0; } //!< Default ctor.

    //! Complete ctor.
    t_cell(const t_uv& p, const double cellSize)
    {
      for ( int k = 0; k < 2; ++k )
      {
        double val = p.Coord(k) / cellSize;

        // If the value of index is greater than INT_MAX, it is decreased
        // correspondingly for the value of INT_MAX. If the value of index is
        // less than INT_MIN, it is increased correspondingly for the absolute
        // value of INT_MIN.
        indices[k] = long((val > INT_MAX - 1) ? fmod(val, (double) INT_MAX) : (val < INT_MIN + 1) ? fmod(val, (double) INT_MIN) : val);
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

  //! Initializes this object with descriptors of basis spline functions.
  //! \param[in] U knot vector in U direction.
  //! \param[in] p spline degree in U direction.
  //! \param[in] V knot vector in V direction.
  //! \param[in] q spline degree in V direction.
  //! \param[in] k serial index of the basis Nk coeff.
  //! \param[in] n index of the last pole in V direction.
  void init(const std::vector<double>& U,
            const int                  p,
            const std::vector<double>& V,
            const int                  q,
            const int                  k,
            const int                  n)
  {
    // Initialize Ni and Nj.
    m_Ni.U = U;
    m_Ni.p = p;
    //
    m_Nj.V = V;
    m_Nj.q = q;
    //
    bspl::PairIndicesFromSerial(k, n + 1, m_Ni.i, m_Nj.j);
  }

protected:

  //! Cached values of \f$N_k(u,v)\f$ for certain pairs of \f$(u,v)\f$
  //! parameters. Such caching technique is effective because the functions
  //! \f$N_k(u,v)\f$ are evaluated always in the same predefined points used
  //! by Gauss integration scheme.
  std::unordered_map<t_cell, t_values, t_cell::hasher> m_cells;

  t_ptr<t_alloc2d> m_alloc; //!< Shared memory allocator.

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

}

#endif
