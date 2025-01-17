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
#include <mobius/poly_Jacobian.h>

using namespace mobius;

//-----------------------------------------------------------------------------

bool poly_Jacobian::Compute(const t_xyz& P0,
                            const t_xyz& P1,
                            const t_xyz& P2,
                            const int    zeroBasedNodeId,
                            t_uv&        p0,
                            t_uv&        p1,
                            t_uv&        p2,
                            double       J[][2],
                            double&      J_det,
                            double&      J_det_normalized)
{
  // Calculate link vectors.
  const t_xyz L0 = P2 - P1;
  const t_xyz L1 = P0 - P2;
  const t_xyz L2 = P1 - P0;
  //
  const double L0_mod = L0.Modulus();
  const double L1_mod = L1.Modulus();
  const double L2_mod = L2.Modulus();

  // Check degeneracy.
  if ( L0_mod < core_Precision::Resolution3D() ||
       L1_mod < core_Precision::Resolution3D() ||
       L2_mod < core_Precision::Resolution3D() )
  {
    return false;
  }

  // Calculate u and v.
  const double u = (L1_mod*L1_mod - L0_mod*L0_mod + L2_mod*L2_mod) / (2*L2_mod);
  //
  if ( L1_mod*L1_mod - u*u < 0 )
    return false;
  //
  const double v = std::sqrt(L1_mod*L1_mod - u*u);

  // Set Jacobian.
  if ( zeroBasedNodeId == 0 )
  {
    J[0][0] = L2_mod;
    J[0][1] = u;
    J[1][0] = 0.0;
    J[1][1] = v;
  }
  else if ( zeroBasedNodeId == 1 )
  {
    J[0][0] = u - L2_mod;
    J[0][1] = -L2_mod;
    J[1][0] = v;
    J[1][1] = 0.0;
  }
  else if ( zeroBasedNodeId == 2 )
  {
    J[0][0] = -u;
    J[0][1] = L2_mod - u;
    J[1][0] = -v;
    J[1][1] = -v;
  }
  else
    return false;

  // Map nodes to 2D.
  p0 = t_uv(0.0,    0.0);
  p1 = t_uv(L2_mod, 0.0);
  p2 = t_uv(u,      v);

  // Compute the determinant
  J_det = J[0][0]*J[1][1] - J[0][1]*J[1][0];

  const double Lmax_mod = std::max( L0_mod*L1_mod, std::max(L0_mod*L2_mod, L1_mod*L2_mod) );
  J_det_normalized = 2*J_det/( Lmax_mod*std::sqrt(3) );

  return true;
}
