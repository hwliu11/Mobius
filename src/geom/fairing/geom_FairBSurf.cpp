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
#include <mobius/geom_FairBSurfBl.h>

// Core includes
#include <mobius/core_Integral.h>
#include <mobius/core_Timer.h>

// Standard includes
#include <map>

// Eigen includes
#pragma warning(disable : 4701 4702)
#include <Eigen/Dense>
#pragma warning(default : 4701 4702)

//-----------------------------------------------------------------------------

#undef COUT_DEBUG
#if defined COUT_DEBUG
  #pragma message("===== warning: COUT_DEBUG is enabled")
#endif

//-----------------------------------------------------------------------------

namespace mobius {

  //! Convenience function to integrate by knot spans.
  double Integral(geom_FairBSurfCoeff*       pCoeff,
                  const std::vector<double>& U,
                  const std::vector<double>& V,
                  const int                  p,
                  const int                  q)
  {
    const int NUM_GAUSS_PT_U = max(p, 3);
    const int NUM_GAUSS_PT_V = max(q, 3);

    // According to the local support property of B-spline basis functions
    // (see for example P2.1 at p. 55 in "The NURBS Book"), not all spans
    // are effective.
    int iFirst = 0, iLast = int( U.size() - 1 ), // Global range.
        jFirst = 0, jLast = int( V.size() - 1 ); // Global range.
    //
    pCoeff->GetSupportSpans(iFirst, iLast, jFirst, jLast);

    // Integrate in each span individually for better accuracy.
    double result = 0;
    for ( size_t i = iFirst; i < iLast; ++i )
    {
      if ( U[i] == U[i+1] ) continue; // Skip multiple knots.

      for ( size_t j = jFirst; j < jLast; ++j )
      {
        if ( V[j] == V[j+1] ) continue; // Skip multiple knots.

        // Gauss integration in each knot span.
        const double
          gaussVal = core_Integral::gauss::Compute(pCoeff,
                                                   U[i], U[i+1],
                                                   V[j], V[j+1],
                                                   NUM_GAUSS_PT_U, NUM_GAUSS_PT_V);
        //
        result += gaussVal;
      }
    }

    return result;
  }
}

//-----------------------------------------------------------------------------

mobius::geom_FairBSurf::geom_FairBSurf(const t_ptr<t_bsurf>& surface,
                                       const double          lambda,
                                       core_ProgressEntry    progress,
                                       core_PlotterEntry     plotter)
: core_OPERATOR(progress, plotter)
{
  m_inputSurf  = surface;
  m_fLambda    = lambda;
  m_iNumPolesU = int( surface->GetPoles().size() );
  m_iNumPolesV = int( surface->GetPoles()[0].size() );
}

//-----------------------------------------------------------------------------

bool mobius::geom_FairBSurf::Perform()
{
  // Dimension of the fairing problem is equal to the number of control
  // points which are not constrained.
  const int nPinned = this->GetNumPinnedPoles();
  const int nPoles  = m_iNumPolesU*m_iNumPolesV;
  const int dim     = nPoles - nPinned;

  std::cout << "Dimension: " << dim << std::endl;

  // Prepare working variables.
  const std::vector<double>& U = m_inputSurf->GetKnots_U();
  const std::vector<double>& V = m_inputSurf->GetKnots_V();
  const int                  p = m_inputSurf->GetDegree_U();
  const int                  q = m_inputSurf->GetDegree_V();

  // Prepare reusable memory blocks for running sub-routines efficiently.
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
  this->prepareNk(sharedAlloc);

  std::cout << "Computing matrix A..." << std::endl;

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

      geom_FairBSurfAkl A_kl_func(k, l, m_fLambda, m_Nk);

      // Compute integral.
      const double val = Integral(&A_kl_func, U, V, p, q);
      eigen_A_mx(r, c) = eigen_A_mx(c, r) = val;
      c++;
    }
    rkMap[r] = k;
    r++;

    std::cout << "A " << r << " done" << std::endl;
  }

#if defined COUT_DEBUG
  std::cout << "Here is the matrix A:\n" << eigen_A_mx << std::endl;
#endif

  std::cout << "Computing matrix b..." << std::endl;

  // Initialize vector of right hand side.
  Eigen::MatrixXd eigen_B_mx(dim, 3);
  r = 0;
  for ( int k = 0; k < nPoles; ++k )
  {
    if ( this->IsPinned(k) )
      continue;

    geom_FairBSurfBl rhs_x(m_inputSurf, 0, k, m_Nk, m_fLambda, sharedAlloc);
    //
    geom_FairBSurfBl rhs_y(m_inputSurf, 1, k, m_Nk, m_fLambda, sharedAlloc);
    //
    geom_FairBSurfBl rhs_z(m_inputSurf, 2, k, m_Nk, m_fLambda, sharedAlloc);

    // Compute integrals.
    const double val_x = Integral(&rhs_x, U, V, p, q);
    const double val_y = Integral(&rhs_y, U, V, p, q);
    const double val_z = Integral(&rhs_z, U, V, p, q);
    //
    eigen_B_mx(r, 0) = -val_x;
    eigen_B_mx(r, 1) = -val_y;
    eigen_B_mx(r, 2) = -val_z;
    //
    r++;
  }

#if defined COUT_DEBUG
  std::cout << "Here is the matrix B:\n" << eigen_B_mx << std::endl;
#endif

  std::cout << "Solving linear system..." << std::endl;

  // Solve.
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> QR(eigen_A_mx);
  Eigen::MatrixXd eigen_X_mx = QR.solve(eigen_B_mx);

#if defined COUT_DEBUG
  std::cout << "Here is the matrix X (solution):\n" << eigen_X_mx << std::endl;
#endif

  std::cout << "Constructing result..." << std::endl;

  // Prepare a copy of the surface to serve as a result.
  m_resultSurf = m_inputSurf->Copy();

  // Apply perturbations to poles.
  const std::vector< std::vector<t_xyz> >& poles = m_resultSurf->GetPoles();
  //
  for ( int r = 0; r < dim; ++r )
  {
    int k = rkMap[r];
    int i, j;
    this->GetIJ(k, i, j);

    const t_xyz& P = poles[i][j];
    t_xyz        D = t_xyz( eigen_X_mx(r, 0), eigen_X_mx(r, 1), eigen_X_mx(r, 2) );
    //
    m_resultSurf->SetPole(i, j, P + D);
  }

  m_plotter.REDRAW_SURFACE("faired", m_resultSurf, MobiusColor_Green);

  return true;
}

//-----------------------------------------------------------------------------

void mobius::geom_FairBSurf::prepareNk(t_ptr<t_alloc2d> alloc)
{
  const int nPoles = m_iNumPolesU*m_iNumPolesV;

  for ( int k = 0; k < nPoles; ++k )
  {
    // Prepare evaluator for N_k(u,v).
    t_ptr<geom_BSurfNk>
      Nk = new geom_BSurfNk(m_inputSurf, k, alloc);
    //
    m_Nk.push_back(Nk);
  }
}
