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

namespace mobius {

//! \ingroup MOBIUS_POLY
//!
//! Utility to analyze Jacobians of mesh elements.
class poly_Jacobian
{
public:

  //! Ctor accepting mesh and diagnostic tools.
  //!
  //! \param mesh     [in] mesh to analyze.
  //! \param notifier [in] progress notifier.
  //! \param plotter  [in] imperative plotter.
  algoMesh_EXPORT
    algoMesh_Jacobian(const Handle(Poly_Triangulation)&       mesh,
                      const Handle(ActAPI_IProgressNotifier)& notifier,
                      const Handle(ActAPI_IPlotter)&          plotter);

public:

  //! Calculates Jacobian for the triangle with the given 1-based index.
  //!
  //! \param oneBasedId       [in]  1-based ID of the mesh element to check.
  //! \param zeroBasedNodeId  [in]  0-based ID of the node where to compute the
  //!                               Jacobian matrix.
  //! \param p0               [out] P0 mapped to 2D.
  //! \param p1               [out] P1 mapped to 2D.
  //! \param p2               [out] P2 mapped to 2D.
  //! \param J                [out] resulting Jacobian matrix 2x2.
  //! \param J_det            [out] determinant of the Jacobian matrix.
  //! \param J_det_normalized [out] normalized determinant.
  //!
  //! \return true in case of success, false -- otherwise.
  algoMesh_EXPORT bool
    Compute(const int    oneBasedElemId,
            const int    zeroBasedNodeId,
            gp_XY&       p0,
            gp_XY&       p1,
            gp_XY&       p2,
            math_Matrix& J,
            double&      J_det,
            double&      J_det_normalized) const;

  //! Calculates Jacobian for the given triangle.
  //!
  //! \param elem             [in]  mesh element to check.
  //! \param zeroBasedNodeId  [in]  0-based ID of the node where to compute the
  //!                               Jacobian matrix.
  //! \param p0               [out] P0 mapped to 2D.
  //! \param p1               [out] P1 mapped to 2D.
  //! \param p2               [out] P2 mapped to 2D.
  //! \param J                [out] resulting Jacobian matrix 2x2.
  //! \param J_det            [out] determinant of the Jacobian matrix.
  //! \param J_det_normalized [out] normalized determinant.
  //!
  //! \return true in case of success, false -- otherwise.
  algoMesh_EXPORT bool
    Compute(const Poly_Triangle& elem,
            const int            zeroBasedNodeId,
            gp_XY&               p0,
            gp_XY&               p1,
            gp_XY&               p2,
            math_Matrix&         J,
            double&              J_det,
            double&              J_det_normalized) const;

protected:

  Handle(Poly_Triangulation) m_mesh; //!< Mesh to analyze.

};

};

#endif
