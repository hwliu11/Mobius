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

// Own include
#include <mobius/geom_FairBSurf.h>

// Geom includes
#include <mobius/geom_FairBSurfAkl.h>
#include <mobius/geom_FairBSurfBk.h>

// Core includes
#include <mobius/core_Integral.h>
#include <mobius/core_Timer.h>

// Standard includes
#include <map>

// Eigen includes
#pragma warning(push, 0)
#pragma warning(disable : 4702 4701)
#include <Eigen/Dense>
#pragma warning(default : 4702 4701)
#pragma warning(pop)

//-----------------------------------------------------------------------------

#undef COUT_DEBUG
#if defined COUT_DEBUG
  #pragma message("===== warning: COUT_DEBUG is enabled")
#endif

//-----------------------------------------------------------------------------

mobius::geom_FairBSurf::geom_FairBSurf(const t_ptr<t_bsurf>& surface,
                                       const double          lambda,
                                       core_ProgressEntry    progress,
                                       core_PlotterEntry     plotter)
: geom_OptimizeBSurfBase(surface, progress, plotter)
{
  m_fLambda = lambda;
}

//-----------------------------------------------------------------------------

bool mobius::geom_FairBSurf::Perform()
{
  // Dimension of the fairing problem is equal to the number of control
  // points which are not constrained.
  const int nPinned = this->GetNumPinnedPoles();
  const int nPoles  = m_iNumPolesU*m_iNumPolesV;
  const int dim     = nPoles - nPinned;

#if defined COUT_DEBUG
  std::cout << "Dimension: " << dim << std::endl;
#endif

  // Prepare working variables.
  const std::vector<double>& U = m_initSurf->GetKnots_U();
  const std::vector<double>& V = m_initSurf->GetKnots_V();
  const int                  p = m_initSurf->GetDegree_U();
  const int                  q = m_initSurf->GetDegree_V();

  // Prepare memory arena (reusable memory blocks for running sub-routines
  // efficiently).
  t_ptr<t_alloc2d> sharedAlloc = new t_alloc2d;
  //
  sharedAlloc->Allocate(3, p + 1, true); // memBlockSurf_EffectiveNDersUResult
  sharedAlloc->Allocate(3, q + 1, true); // memBlockSurf_EffectiveNDersVResult
  sharedAlloc->Allocate(2, 3,     true); // memBlockSurf_EffectiveNDersUInternal
  sharedAlloc->Allocate(2, 3,     true); // memBlockSurf_EffectiveNDersVInternal
  sharedAlloc->Allocate(3, p + 1, true); // memBlockSurf_BSplineSurfEvalD2U
  sharedAlloc->Allocate(3, q + 1, true); // memBlockSurf_BSplineSurfEvalD2V
  sharedAlloc->Allocate(2, 3,     true); // memBlockSurf_BSplineSurfEvalInternal

  // Prepare N_k(u,v) evaluators which will be shared in all integration
  // requests to take advantage of caching.
  if ( !this->prepareNk(sharedAlloc) )
  {
    m_progress.SendLogMessage(MobiusErr(Normal) << "Failed to prepare basis N_k functions.");
    return false;
  }

#if defined COUT_DEBUG
  std::cout << "Computing matrix A..." << std::endl;
#endif

  // Mapping between row indices of the linear system and serial indices of
  // the perturbed control points.
  std::map<int, int> rkMap;

  // Initialize matrix of left-hand-side coefficients.
  int r = 0;
  Eigen::MatrixXd eigen_A_mx(dim, dim);
  for ( int k = 0; k < nPoles; ++k )
  {
    if ( this->IsPinned(k) )
      continue;

    // Fill upper triangle and populate the matrix symmetrically.
    int c = r;
    for ( int l = k; l < nPoles; ++l )
    {
      if ( this->IsPinned(l) )
        continue;

      geom_FairBSurfAkl A_kl_func(k, l, m_fLambda, m_Nk, false);

      // Compute integral.
      const double val = A_kl_func.Integral(U, V, p, q);
      eigen_A_mx(r, c) = eigen_A_mx(c, r) = val;
      c++;
    }
    rkMap[r] = k;
    r++;

#if defined COUT_DEBUG
    std::cout << "A " << r << " done" << std::endl;
#endif
  }

#if defined COUT_DEBUG
  std::cout << "Here is the matrix A:\n" << eigen_A_mx << std::endl;
  std::cout << "Computing matrix b..." << std::endl;
#endif

  // Initialize vector of right hand side.
  Eigen::MatrixXd eigen_B_mx(dim, 3);
  r = 0;
  for ( int k = 0; k < nPoles; ++k )
  {
    if ( this->IsPinned(k) )
      continue;

    geom_FairBSurfBk rhs_x(m_initSurf, 0, k, m_Nk, m_fLambda, sharedAlloc);
    geom_FairBSurfBk rhs_y(m_initSurf, 1, k, m_Nk, m_fLambda, sharedAlloc);
    geom_FairBSurfBk rhs_z(m_initSurf, 2, k, m_Nk, m_fLambda, sharedAlloc);

    // Compute integrals.
    const double val_x = rhs_x.Integral(U, V, p, q);
    const double val_y = rhs_y.Integral(U, V, p, q);
    const double val_z = rhs_z.Integral(U, V, p, q);
    //
    eigen_B_mx(r, 0) = -val_x;
    eigen_B_mx(r, 1) = -val_y;
    eigen_B_mx(r, 2) = -val_z;
    //
    r++;
  }

#if defined COUT_DEBUG
  std::cout << "Here is the matrix B:\n" << eigen_B_mx << std::endl;
  std::cout << "Solving linear system..." << std::endl;
#endif

  // Solve.
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> QR(eigen_A_mx);
  Eigen::MatrixXd eigen_X_mx = QR.solve(eigen_B_mx);

#if defined COUT_DEBUG
  std::cout << "Here is the matrix X (solution):\n" << eigen_X_mx << std::endl;
  std::cout << "Constructing result..." << std::endl;
#endif

  // Prepare a copy of the surface to serve as a result.
  m_resultSurf = m_initSurf->Copy();

  // Apply perturbations to poles.
  const std::vector< std::vector<t_xyz> >& poles = m_resultSurf->GetPoles();
  //
  for ( int rr = 0; rr < dim; ++rr )
  {
    int k = rkMap[rr];
    int i, j;
    this->GetIJ(k, i, j);

    const t_xyz& P = poles[i][j];
    t_xyz        D = t_xyz( eigen_X_mx(rr, 0), eigen_X_mx(rr, 1), eigen_X_mx(rr, 2) );
    //
    m_resultSurf->SetPole(i, j, P + D);
  }

  m_plotter.REDRAW_SURFACE("faired", m_resultSurf, MobiusColor_Green);

  return true;
}
