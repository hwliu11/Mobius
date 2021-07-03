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

// Own include
#include <cadpro/algoMesh_Jacobian.h>

#undef COUT_DEBUG
#if defined COUT_DEBUG
  #pragma message("===== warning: COUT_DEBUG is enabled")
#endif

//-----------------------------------------------------------------------------

cadpro::algoMesh_Jacobian::algoMesh_Jacobian(const Handle(Poly_Triangulation)&       mesh,
                                             const Handle(ActAPI_IProgressNotifier)& notifier,
                                             const Handle(ActAPI_IPlotter)&          plotter)
: ActAPI_IAlgorithm(notifier, plotter)
{
  m_mesh = mesh;
}

//-----------------------------------------------------------------------------

bool cadpro::algoMesh_Jacobian::Compute(const int    oneBasedElemId,
                                        const int    zeroBasedNodeId,
                                        gp_XY&       p0,
                                        gp_XY&       p1,
                                        gp_XY&       p2,
                                        math_Matrix& J,
                                        double&      J_det,
                                        double&      J_det_normalized) const
{
  if ( !oneBasedElemId || m_mesh->NbTriangles() < oneBasedElemId)
    return false;

  // Get element
  const Poly_Triangle& elem = m_mesh->Triangle(oneBasedElemId);

  // Compute for element
  return this->Compute(elem, zeroBasedNodeId, p0, p1, p2, J, J_det, J_det_normalized);
}

//-----------------------------------------------------------------------------

bool cadpro::algoMesh_Jacobian::Compute(const Poly_Triangle& elem,
                                        const int            zeroBasedNodeId,
                                        gp_XY&               p0,
                                        gp_XY&               p1,
                                        gp_XY&               p2,
                                        math_Matrix&         J,
                                        double&              J_det,
                                        double&              J_det_normalized) const
{
  // Get nodes
  int n0, n1, n2;
  elem.Get(n0, n1, n2);

  // Get nodes
  const gp_Pnt& P0 = m_mesh->Node(n0);
  const gp_Pnt& P1 = m_mesh->Node(n1);
  const gp_Pnt& P2 = m_mesh->Node(n2);

  // Calculate link vectors
  const gp_Vec L0 = P2.XYZ() - P1.XYZ();
  const gp_Vec L1 = P0.XYZ() - P2.XYZ();
  const gp_Vec L2 = P1.XYZ() - P0.XYZ();
  //
  const double L0mod = L0.Magnitude();
  const double L1mod = L1.Magnitude();
  const double L2mod = L2.Magnitude();

  // Check degeneracy
  if ( L0mod < RealEpsilon() ||
       L1mod < RealEpsilon() ||
       L2mod < RealEpsilon() )
    return false;

  // Calculate u and v
  const double u = ( Square(L1mod) - Square(L0mod) + Square(L2mod) ) / (2*L2mod);
  //
  if (Square(L1mod) - Square(u) < 0)
    return false;
  //
  const double v =  Sqrt(Square(L1mod) - Square(u));

  // Check if the passed matrix has capacity to store 2x2 grid
  if ( J.ColNumber() < 2 || J.RowNumber() < 2 )
    return false;

  // Set Jacobian
  if ( zeroBasedNodeId == 0 )
  {
    J( J.LowerRow(),     J.LowerCol() )     = L2mod;
    J( J.LowerRow(),     J.LowerCol() + 1 ) = u;
    J( J.LowerRow() + 1, J.LowerCol() )     = 0.0;
    J( J.LowerRow() + 1, J.LowerCol() + 1 ) = v;
  }
  else if ( zeroBasedNodeId == 1 )
  {
    J( J.LowerRow(),     J.LowerCol() )     = u - L2mod;
    J( J.LowerRow(),     J.LowerCol() + 1 ) = -L2mod;
    J( J.LowerRow() + 1, J.LowerCol() )     = v;
    J( J.LowerRow() + 1, J.LowerCol() + 1 ) = 0.0;
  }
  else if ( zeroBasedNodeId == 2 )
  {
    J( J.LowerRow(),     J.LowerCol() )     = -u;
    J( J.LowerRow(),     J.LowerCol() + 1 ) = L2mod - u;
    J( J.LowerRow() + 1, J.LowerCol() )     = -v;
    J( J.LowerRow() + 1, J.LowerCol() + 1 ) = -v;
  }
  else
    return false;

  // Map nodes to 2D
  p0 = gp_XY(0.0,   0.0);
  p1 = gp_XY(L2mod, 0.0);
  p2 = gp_XY(u,     v);

  // Compute the determinant
  J_det = J( J.LowerRow(), J.LowerCol()    )*J( J.LowerRow() + 1, J.LowerCol() + 1 )
        - J( J.LowerRow(), J.LowerCol() + 1)*J( J.LowerRow() + 1, J.LowerCol()     );

  const double LLmod = Max( L0mod*L1mod, Max(L0mod*L2mod, L1mod*L2mod) );
  J_det_normalized = 2*J_det/( LLmod*Sqrt(3) );

  return true;
}
