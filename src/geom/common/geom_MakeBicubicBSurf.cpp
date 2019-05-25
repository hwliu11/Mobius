//-----------------------------------------------------------------------------
// Created on: 14 December 2018
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

// Own include
#include <mobius/geom_MakeBicubicBSurf.h>

// Eigen includes
#pragma warning(disable : 4701 4702)
#include <Eigen/Dense>
#pragma warning(default : 4701 4702)

//-----------------------------------------------------------------------------

#define COUT_DEBUG
#if defined COUT_DEBUG
  #pragma message("===== warning: COUT_DEBUG is enabled")
#endif

//-----------------------------------------------------------------------------

mobius::geom_MakeBicubicBSurf::geom_MakeBicubicBSurf(const t_xyz&       S00,
                                                     const t_xyz&       S01,
                                                     const t_xyz&       S10,
                                                     const t_xyz&       S11,
                                                     const t_xyz&       dS_du00,
                                                     const t_xyz&       dS_du01,
                                                     const t_xyz&       dS_du10,
                                                     const t_xyz&       dS_du11,
                                                     const t_xyz&       dS_dv00,
                                                     const t_xyz&       dS_dv01,
                                                     const t_xyz&       dS_dv10,
                                                     const t_xyz&       dS_dv11,
                                                     const t_xyz&       d2S_dudv00,
                                                     const t_xyz&       d2S_dudv01,
                                                     const t_xyz&       d2S_dudv10,
                                                     const t_xyz&       d2S_dudv11,
                                                     core_ProgressEntry progress,
                                                     core_PlotterEntry  plotter)
: core_OPERATOR (progress, plotter),
  m_S00         (S00),
  m_S01         (S01),
  m_S10         (S10),
  m_S11         (S11),
  m_dS_du00     (dS_du00),
  m_dS_du01     (dS_du01),
  m_dS_du10     (dS_du10),
  m_dS_du11     (dS_du11),
  m_dS_dv00     (dS_dv00),
  m_dS_dv01     (dS_dv01),
  m_dS_dv10     (dS_dv10),
  m_dS_dv11     (dS_dv11),
  m_d2S_dudv00  (d2S_dudv00),
  m_d2S_dudv01  (d2S_dudv01),
  m_d2S_dudv10  (d2S_dudv10),
  m_d2S_dudv11  (d2S_dudv11)
{}

//-----------------------------------------------------------------------------

bool mobius::geom_MakeBicubicBSurf::Perform()
{
  // Prepare working variables.
  const int                  dim = 16;
  const std::vector<double>& U   = {0, 0, 0, 0, 1, 1, 1, 1};
  const std::vector<double>& V   = {0, 0, 0, 0, 1, 1, 1, 1};
  const int                  p   = 3;
  const int                  q   = 3;

  // Initialize matrices of the linear system to solve.
  Eigen::MatrixXd eigen_A_mx(dim, dim);
  Eigen::MatrixXd eigen_B_mx(dim, 3);

  // Prepare coefficient evaluators.
  std::vector<geom_BSurfNk> Nk;
  //
  for ( int k = 0; k < dim; ++k )
    Nk.push_back( geom_BSurfNk(U, p, V, q, k, 3, NULL) );

  // Row 0.
  {
    const int row = 0;

    // Fill RHS.
    eigen_B_mx(row, 0) = m_S00.X();
    eigen_B_mx(row, 1) = m_S00.Y();
    eigen_B_mx(row, 2) = m_S00.Z();

    // Fill coefficients.
    for ( int k = 0; k < dim; ++k )
    {
      double N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2;
      //
      Nk[k].Eval_D2(0., 0., N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2);
      //
      eigen_A_mx(row, k) = N;
    }
  }

  // Row 1.
  {
    const int row = 1;

    // Fill RHS.
    eigen_B_mx(row, 0) = m_S01.X();
    eigen_B_mx(row, 1) = m_S01.Y();
    eigen_B_mx(row, 2) = m_S01.Z();

    // Fill coefficients.
    for ( int k = 0; k < dim; ++k )
    {
      double N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2;
      //
      Nk[k].Eval_D2(0., 1., N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2);
      //
      eigen_A_mx(row, k) = N;
    }
  }

  // Row 2.
  {
    const int row = 2;

    // Fill RHS.
    eigen_B_mx(row, 0) = m_S10.X();
    eigen_B_mx(row, 1) = m_S10.Y();
    eigen_B_mx(row, 2) = m_S10.Z();

    // Fill coefficients.
    for ( int k = 0; k < dim; ++k )
    {
      double N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2;
      //
      Nk[k].Eval_D2(1., 0., N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2);
      //
      eigen_A_mx(row, k) = N;
    }
  }

  // Row 3.
  {
    const int row = 3;

    // Fill RHS.
    eigen_B_mx(row, 0) = m_S11.X();
    eigen_B_mx(row, 1) = m_S11.Y();
    eigen_B_mx(row, 2) = m_S11.Z();

    // Fill coefficients.
    for ( int k = 0; k < dim; ++k )
    {
      double N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2;
      //
      Nk[k].Eval_D2(1., 1., N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2);
      //
      eigen_A_mx(row, k) = N;
    }
  }

  // Row 4.
  {
    const int row = 4;

    // Fill RHS.
    eigen_B_mx(row, 0) = m_dS_du00.X();
    eigen_B_mx(row, 1) = m_dS_du00.Y();
    eigen_B_mx(row, 2) = m_dS_du00.Z();

    // Fill coefficients.
    for ( int k = 0; k < dim; ++k )
    {
      double N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2;
      //
      Nk[k].Eval_D2(0., 0., N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2);
      //
      eigen_A_mx(row, k) = dN_dU;
    }
  }

  // Row 5.
  {
    const int row = 5;

    // Fill RHS.
    eigen_B_mx(row, 0) = m_dS_du01.X();
    eigen_B_mx(row, 1) = m_dS_du01.Y();
    eigen_B_mx(row, 2) = m_dS_du01.Z();

    // Fill coefficients.
    for ( int k = 0; k < dim; ++k )
    {
      double N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2;
      //
      Nk[k].Eval_D2(0., 1., N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2);
      //
      eigen_A_mx(row, k) = dN_dU;
    }
  }

  // Row 6.
  {
    const int row = 6;

    // Fill RHS.
    eigen_B_mx(row, 0) = m_dS_du10.X();
    eigen_B_mx(row, 1) = m_dS_du10.Y();
    eigen_B_mx(row, 2) = m_dS_du10.Z();

    // Fill coefficients.
    for ( int k = 0; k < dim; ++k )
    {
      double N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2;
      //
      Nk[k].Eval_D2(1., 0., N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2);
      //
      eigen_A_mx(row, k) = dN_dU;
    }
  }

  // Row 7.
  {
    const int row = 7;

    // Fill RHS.
    eigen_B_mx(row, 0) = m_dS_du11.X();
    eigen_B_mx(row, 1) = m_dS_du11.Y();
    eigen_B_mx(row, 2) = m_dS_du11.Z();

    // Fill coefficients.
    for ( int k = 0; k < dim; ++k )
    {
      double N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2;
      //
      Nk[k].Eval_D2(1., 1., N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2);
      //
      eigen_A_mx(row, k) = dN_dU;
    }
  }

  // Row 8.
  {
    const int row = 8;

    // Fill RHS.
    eigen_B_mx(row, 0) = m_dS_dv00.X();
    eigen_B_mx(row, 1) = m_dS_dv00.Y();
    eigen_B_mx(row, 2) = m_dS_dv00.Z();

    // Fill coefficients.
    for ( int k = 0; k < dim; ++k )
    {
      double N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2;
      //
      Nk[k].Eval_D2(0., 0., N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2);
      //
      eigen_A_mx(row, k) = dN_dV;
    }
  }

  // Row 9.
  {
    const int row = 9;

    // Fill RHS.
    eigen_B_mx(row, 0) = m_dS_dv01.X();
    eigen_B_mx(row, 1) = m_dS_dv01.Y();
    eigen_B_mx(row, 2) = m_dS_dv01.Z();

    // Fill coefficients.
    for ( int k = 0; k < dim; ++k )
    {
      double N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2;
      //
      Nk[k].Eval_D2(0., 1., N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2);
      //
      eigen_A_mx(row, k) = dN_dV;
    }
  }

  // Row 10.
  {
    const int row = 10;

    // Fill RHS.
    eigen_B_mx(row, 0) = m_dS_dv10.X();
    eigen_B_mx(row, 1) = m_dS_dv10.Y();
    eigen_B_mx(row, 2) = m_dS_dv10.Z();

    // Fill coefficients.
    for ( int k = 0; k < dim; ++k )
    {
      double N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2;
      //
      Nk[k].Eval_D2(1., 0., N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2);
      //
      eigen_A_mx(row, k) = dN_dV;
    }
  }

  // Row 11.
  {
    const int row = 11;

    // Fill RHS.
    eigen_B_mx(row, 0) = m_dS_dv11.X();
    eigen_B_mx(row, 1) = m_dS_dv11.Y();
    eigen_B_mx(row, 2) = m_dS_dv11.Z();

    // Fill coefficients.
    for ( int k = 0; k < dim; ++k )
    {
      double N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2;
      //
      Nk[k].Eval_D2(1., 1., N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2);
      //
      eigen_A_mx(row, k) = dN_dV;
    }
  }

  // Row 12.
  {
    const int row = 12;

    // Fill RHS.
    eigen_B_mx(row, 0) = m_d2S_dudv00.X();
    eigen_B_mx(row, 1) = m_d2S_dudv00.Y();
    eigen_B_mx(row, 2) = m_d2S_dudv00.Z();

    // Fill coefficients.
    for ( int k = 0; k < dim; ++k )
    {
      double N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2;
      //
      Nk[k].Eval_D2(0., 0., N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2);
      //
      eigen_A_mx(row, k) = d2N_dUV;
    }
  }

  // Row 13.
  {
    const int row = 13;

    // Fill RHS.
    eigen_B_mx(row, 0) = m_d2S_dudv01.X();
    eigen_B_mx(row, 1) = m_d2S_dudv01.Y();
    eigen_B_mx(row, 2) = m_d2S_dudv01.Z();

    // Fill coefficients.
    for ( int k = 0; k < dim; ++k )
    {
      double N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2;
      //
      Nk[k].Eval_D2(0., 1., N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2);
      //
      eigen_A_mx(row, k) = d2N_dUV;
    }
  }

  // Row 14.
  {
    const int row = 14;

    // Fill RHS.
    eigen_B_mx(row, 0) = m_d2S_dudv10.X();
    eigen_B_mx(row, 1) = m_d2S_dudv10.Y();
    eigen_B_mx(row, 2) = m_d2S_dudv10.Z();

    // Fill coefficients.
    for ( int k = 0; k < dim; ++k )
    {
      double N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2;
      //
      Nk[k].Eval_D2(1., 0., N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2);
      //
      eigen_A_mx(row, k) = d2N_dUV;
    }
  }

  // Row 15.
  {
    const int row = 15;

    // Fill RHS.
    eigen_B_mx(row, 0) = m_d2S_dudv11.X();
    eigen_B_mx(row, 1) = m_d2S_dudv11.Y();
    eigen_B_mx(row, 2) = m_d2S_dudv11.Z();

    // Fill coefficients.
    for ( int k = 0; k < dim; ++k )
    {
      double N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2;
      //
      Nk[k].Eval_D2(1., 1., N, dN_dU, dN_dV, d2N_dU2, d2N_dUV, d2N_dV2);
      //
      eigen_A_mx(row, k) = d2N_dUV;
    }
  }

#if defined COUT_DEBUG
  std::cout << "Here is the matrix A:\n" << eigen_A_mx << std::endl;
  std::cout << "Here is the matrix b:\n" << eigen_B_mx << std::endl;
#endif

  std::cout << "Solving linear system..." << std::endl;

  // Solve.
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> QR(eigen_A_mx);
  Eigen::MatrixXd eigen_X_mx = QR.solve(eigen_B_mx);

#if defined COUT_DEBUG
  std::cout << "Here is the matrix X (solution):\n" << eigen_X_mx << std::endl;
#endif

  std::cout << "Constructing result..." << std::endl;

  // Prepare poles.
  std::vector< std::vector<t_xyz> > poles = { { t_xyz(), t_xyz(), t_xyz(), t_xyz() },
                                              { t_xyz(), t_xyz(), t_xyz(), t_xyz() },
                                              { t_xyz(), t_xyz(), t_xyz(), t_xyz() },
                                              { t_xyz(), t_xyz(), t_xyz(), t_xyz() } };
  //
  for ( int k = 0; k < dim; ++k )
  {
    int i, j;
    bspl::PairIndicesFromSerial(k, 4, i, j);

    t_xyz P( eigen_X_mx(k, 0), eigen_X_mx(k, 1), eigen_X_mx(k, 2) );
    //
    poles[i][j] = P;
  }

  // Construct a surface.
  m_surf = new t_bsurf(poles, U, V, p, q);

  return true;
}
