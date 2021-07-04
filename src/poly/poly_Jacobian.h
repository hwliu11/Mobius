//-----------------------------------------------------------------------------
// Created on: 03 July 2021
//-----------------------------------------------------------------------------
// Copyright (c) 2021-present, Sergey Slyadnev
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

#ifndef poly_Jacobian_h
#define poly_Jacobian_h

// poly includes
#include <mobius/poly_Mesh.h>

// core includes
#include <mobius/core_UV.h>

namespace mobius {

//! \ingroup MOBIUS_POLY
//!
//! Utility to analyze Jacobians of mesh elements. See
//!
//!   [Shivanna et al (2010). An Analytical Framework for Quadrilateral Surface Mesh Improvement with an Underlying Triangulated Surface Definition. International Meshing Roundtable, 335-350.]
//!   [Stimpson et al (2007). The Verdict Geometric Quality Library, Ch. 4.9]
//!
//! According to [Shivanna et al. 2010], the Jacobian of a triangle in 3D/physical space is a
//! 3x2 matrix and that of a triangle in 2D/computational/parametric space is a 2x2 matrix.
//! To calculate the quality metric of a triangle in 3D, the triangle is rigidly transformed
//! so that all the nodes lie in a 2D plane. This transformation does not change the quality
//! metric, as the determinant and the condition number of the Jacobian are orientation invariant.
class poly_Jacobian
{
public:

  //! Computes scaled Jacobian
  //! \param[in]  P0               the 0-th vertex coordinates.
  //! \param[in]  P1               the 1-st vertex coordinates.
  //! \param[in]  P2               the 2-nd vertex coordinates.
  //! \param[in]  zeroBasedNodeId  the 0-based ID of the node where to compute the
  //!                              Jacobian matrix.
  //! \param[out] p0               the P0 mapped to 2D.
  //! \param[out] p1               the P1 mapped to 2D.
  //! \param[out] p2               the P2 mapped to 2D.
  //! \param[out] J                the resulting Jacobian matrix 2x2.
  //! \param[out] J_det            the determinant of the Jacobian matrix.
  //! \param[out] J_det_normalized the normalized determinant.
  //!
  //! \return true in case of success, false -- otherwise.
  mobiusPoly_EXPORT static bool
    Compute(const t_xyz& P0,
            const t_xyz& P1,
            const t_xyz& P2,
            const int    zeroBasedNodeId,
            t_uv&        p0,
            t_uv&        p1,
            t_uv&        p2,
            double       J[][2],
            double&      J_det,
            double&      J_det_normalized);

public:

  //! Ctor accepting mesh and diagnostic tools.
  //!
  //! \param[in] mesh the mesh to analyze.
  mobiusPoly_EXPORT
    poly_Jacobian(const t_ptr<poly_Mesh>& mesh);

public:

  //! Calculates Jacobian for the triangle with the given handle.
  //!
  //! \param[in]  ht               the handle of the mesh element to check.
  //! \param[in]  zeroBasedNodeId  the 0-based ID of the node where to compute the
  //!                              Jacobian matrix.
  //! \param[out] p0               the P0 mapped to 2D.
  //! \param[out] p1               the P1 mapped to 2D.
  //! \param[out] p2               the P2 mapped to 2D.
  //! \param[out] J                the resulting Jacobian matrix 2x2.
  //! \param[out] J_det            the determinant of the Jacobian matrix.
  //! \param[out] J_det_normalized the normalized determinant.
  //!
  //! \return true in case of success, false -- otherwise.
  mobiusPoly_EXPORT bool
    Compute(const poly_TriangleHandle ht,
            const int                 zeroBasedNodeId,
            t_uv&                     p0,
            t_uv&                     p1,
            t_uv&                     p2,
            double                    J[][2],
            double&                   J_det,
            double&                   J_det_normalized) const;

  //! Calculates Jacobian for the given triangle.
  //!
  //! \param[in]  elem             the mesh element to check.
  //! \param[in]  zeroBasedNodeId  the 0-based ID of the node where to compute the
  //!                              Jacobian matrix.
  //! \param[out] p0               the P0 mapped to 2D.
  //! \param[out] p1               the P1 mapped to 2D.
  //! \param[out] p2               the P2 mapped to 2D.
  //! \param[out] J                the resulting Jacobian matrix 2x2.
  //! \param[out] J_det            the determinant of the Jacobian matrix.
  //! \param[out] J_det_normalized the normalized determinant.
  //!
  //! \return true in case of success, false -- otherwise.
  mobiusPoly_EXPORT bool
    Compute(const poly_Triangle& elem,
            const int            zeroBasedNodeId,
            t_uv&                p0,
            t_uv&                p1,
            t_uv&                p2,
            double               J[][2],
            double&              J_det,
            double&              J_det_normalized) const;

protected:

  t_ptr<poly_Mesh> m_mesh; //!< Mesh to analyze.

};

}

#endif
