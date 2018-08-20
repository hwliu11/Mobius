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

namespace mobius {

  //! Convenience function to integrate by knot spans.
  double Integral(geom_FairBSurfCoeff*       pCoeff,
                  const std::vector<double>& U,
                  const std::vector<double>& V,
                  const int                  p,
                  const int                  q)
  {
    // (2n-1) for max accuracy on polynomial functions.
    const int NUM_GAUSS_PT_U = 2*p - 1;
    const int NUM_GAUSS_PT_V = 2*q - 1;

    // Integrate in each span individually for better accuracy.
    double result = 0;
    for ( size_t i = 0; i < U.size() - 1; ++i )
    {
      if ( U[i] == U[i+1] ) continue; // Skip multiple knots.

      for ( size_t j = 0; j < V.size() - 1; ++j )
      {
        if ( V[j] == V[j+1] ) continue; // Skip multiple knots.

        // 6-points integration in each knot span.
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

mobius::geom_FairBSurf::geom_FairBSurf(const ptr<bsurf>&  surface,
                                       const double       lambda,
                                       core_ProgressEntry progress,
                                       core_PlotterEntry  plotter)
: core_OPERATOR(progress, plotter)
{
  m_inputSurf  = surface;
  m_fLambda    = lambda;
  m_iNumPolesU = int( surface->Poles().size() );
  m_iNumPolesV = int( surface->Poles()[0].size() );
}

//-----------------------------------------------------------------------------

bool mobius::geom_FairBSurf::Perform()
{
  // Dimension of the fairing problem is equal to the number of control
  // points which are not constrained.
  const int dim = m_iNumPolesU * m_iNumPolesV;

  // Prepare working variables.
  const std::vector<double>& U = m_inputSurf->Knots_U();
  const std::vector<double>& V = m_inputSurf->Knots_V();
  const int                  p = m_inputSurf->Degree_U();
  const int                  q = m_inputSurf->Degree_V();

  // Initialize matrix of left-hand-side coefficients.
  Eigen::MatrixXd eigen_A_mx(dim, dim);
  for ( int r = 0; r < dim; ++r )
  {
    for ( int c = 0; c < dim; ++c )
    {
      geom_FairBSurfAkl A_kl_func(U, V, p, q, r, c, m_iNumPolesV, m_fLambda, NULL);

      // Compute integral.
      const double val = Integral(&A_kl_func, U, V, p, q);
      //
      eigen_A_mx(r, c) = val;
    }
  }

#if defined COUT_DEBUG
  std::cout << "Here is the matrix A:\n" << eigen_A_mx << std::endl;
#endif

  // Initialize vector of right hand side.
  Eigen::MatrixXd eigen_B_mx(dim, 3);
  for ( int r = 0; r < dim; ++r )
  {
    geom_FairBSurfBl rhs_x(m_inputSurf, 0, r, m_iNumPolesV, m_fLambda, NULL);
    //
    geom_FairBSurfBl rhs_y(m_inputSurf, 1, r, m_iNumPolesV, m_fLambda, NULL);
    //
    geom_FairBSurfBl rhs_z(m_inputSurf, 2, r, m_iNumPolesV, m_fLambda, NULL);

    // Compute integrals.
    const double val_x = Integral(&rhs_x, U, V, p, q);
    const double val_y = Integral(&rhs_y, U, V, p, q);
    const double val_z = Integral(&rhs_z, U, V, p, q);
    //
    eigen_B_mx(r, 0) = -val_x;
    eigen_B_mx(r, 1) = -val_y;
    eigen_B_mx(r, 2) = -val_z;
  }

#if defined COUT_DEBUG
  std::cout << "Here is the matrix B:\n" << eigen_B_mx << std::endl;
#endif

  // Solve.
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> QR(eigen_A_mx);
  Eigen::MatrixXd eigen_X_mx = QR.solve(eigen_B_mx);

#if defined COUT_DEBUG
  std::cout << "Here is the matrix X (solution):\n" << eigen_X_mx << std::endl;
#endif

  // Prepare a copy of the surface to serve as a result.
  m_resultSurf = m_inputSurf->Copy();

  // Apply perturbations to poles.
  const std::vector< std::vector<xyz> >& poles = m_resultSurf->Poles();
  //
  for ( int k = 0; k < dim; ++k )
  {
    int i, j;
    this->GetIJ(k, i, j);

    const xyz& P = poles[i][j];
    xyz        D = xyz( eigen_X_mx(k, 0), eigen_X_mx(k, 1), eigen_X_mx(k, 2) );
    //
    m_resultSurf->SetPole(i, j, P + D);
  }

  m_plotter.REDRAW_SURFACE("faired", m_resultSurf.Access(), MobiusColor_Green);

  return true;
}
