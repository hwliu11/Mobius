//-----------------------------------------------------------------------------
// Created on: 03 March 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2018, Sergey Slyadnev
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
#include <mobius/geom_FairBCurve.h>

// Geom includes
#include <mobius/geom_FairBCurveAij.h>
#include <mobius/geom_FairBCurveBj.h>

// Core includes
#include <mobius/core_Integral.h>
#include <mobius/core_Timer.h>

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

#define NUM_CONSTRAINED_POLES_LEADING  1
#define NUM_CONSTRAINED_POLES_TRAILING 1

//-----------------------------------------------------------------------------

mobius::geom_FairBCurve::geom_FairBCurve(const ptr<bcurve>& curve,
                                         const double       lambda,
                                         core_ProgressEntry progress,
                                         core_PlotterEntry  plotter)
: core_OPERATOR(progress, plotter)
{
  m_inputCurve = curve;
  m_fLambda    = lambda;
}

//-----------------------------------------------------------------------------

bool mobius::geom_FairBCurve::Perform()
{
  // Prepare flat sequence of knots and other working vars.
  const std::vector<double>& U        = m_inputCurve->Knots();
  const int                  p        = m_inputCurve->Degree();
  const int                  m        = int( U.size() ) - 1;
  const int                  n        = m - p - 1;
  const int                  dim      = n + 1 - NUM_CONSTRAINED_POLES_LEADING - NUM_CONSTRAINED_POLES_TRAILING;
  const int                  nGaussPt = 2*p - 1;
  int                        nAijEval = 0;

  // Prepare reusable memory blocks for running sub-routines efficiently.
  ptr<alloc2d> sharedAlloc = new alloc2d;
  sharedAlloc->Allocate(3,     p + 1, true); // memBlock_EffectiveNDersResult
  sharedAlloc->Allocate(2,     3,     true); // memBlock_EffectiveNDersInternal
  sharedAlloc->Allocate(p + 1, p + 1, true); // memBlock_BSplineCurveEvalDk

  // Initialize matrix of left-hand-side coefficients.
  Eigen::MatrixXd eigen_A_mx(dim, dim);
  for ( int r = 0; r < dim; ++r )
  {
    for ( int c = 0; c < dim; ++c )
    {
      geom_FairBCurveAij N2(U,
                            p,
                            r + NUM_CONSTRAINED_POLES_LEADING,
                            c + NUM_CONSTRAINED_POLES_LEADING,
                            m_fLambda,
                            sharedAlloc);

      // Compute integral.
      double val = 0;
      for ( size_t k = 0; k < U.size() - 1; ++k )
      {
        if ( U[k] == U[k+1] ) continue; // Skip multiple knots.

        // nGaussPt-points integration in each knot span.
        const double
          gaussVal = core_Integral::gauss::Compute(&N2, U[k], U[k+1], nGaussPt, nAijEval);
        //
        val += gaussVal;
      }

      eigen_A_mx(r, c) = val;
    }
  }

#if defined COUT_DEBUG
  std::cout << "Here is the matrix A:\n" << eigen_A_mx << std::endl;
#endif

  // Initialize vector of right-hand-side coefficients.
  Eigen::MatrixXd eigen_B_mx(dim, 3);
  for ( int r = 0; r < dim; ++r )
  {
    geom_FairBCurveBj rhs_x(m_inputCurve,
                            0,
                            r + NUM_CONSTRAINED_POLES_LEADING,
                            m_fLambda,
                            sharedAlloc);
    //
    geom_FairBCurveBj rhs_y(m_inputCurve,
                            1,
                            r + NUM_CONSTRAINED_POLES_LEADING,
                            m_fLambda,
                            sharedAlloc);
    //
    geom_FairBCurveBj rhs_z(m_inputCurve,
                            2,
                            r + NUM_CONSTRAINED_POLES_LEADING,
                            m_fLambda,
                            sharedAlloc);

    // Compute integrals.
    double val_x = 0;
    for ( size_t k = 0; k < U.size() - 1; ++k )
    {
      if ( U[k] == U[k+1] ) continue; // Skip multiple knots.

      // nGaussPt-points integration in each knot span.
      const double
        gaussVal = core_Integral::gauss::Compute(&rhs_x, U[k], U[k+1], nGaussPt);
      //
      val_x += gaussVal;
    }
    //
    double val_y = 0;
    for ( size_t k = 0; k < U.size() - 1; ++k )
    {
      if ( U[k] == U[k+1] ) continue; // Skip multiple knots.

      // nGaussPt-points integration in each knot span.
      const double
        gaussVal = core_Integral::gauss::Compute(&rhs_y, U[k], U[k+1], nGaussPt);
      //
      val_y += gaussVal;
    }
    //
    double val_z = 0;
    for ( size_t k = 0; k < U.size() - 1; ++k )
    {
      if ( U[k] == U[k+1] ) continue; // Skip multiple knots.

      // nGaussPt-points integration in each knot span.
      const double
        gaussVal = core_Integral::gauss::Compute(&rhs_z, U[k], U[k+1], nGaussPt);
      //
      val_z += gaussVal;
    }

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

  // Prepare a copy of the curve to serve as a result.
  m_resultCurve = m_inputCurve->Copy();

  // Apply perturbations to poles.
  const std::vector<xyz>& poles = m_resultCurve->Poles();
  int                     r     = 0;
  //
  for ( size_t p = 0 + NUM_CONSTRAINED_POLES_LEADING;
        p <= poles.size() - 1 - NUM_CONSTRAINED_POLES_TRAILING;
        ++p, ++r )
  {
    const xyz& P = poles[p];
    xyz        D = xyz( eigen_X_mx(r, 0), eigen_X_mx(r, 1), eigen_X_mx(r, 2) );
    //
    m_resultCurve->SetPole(p, P + D);
  }

#if defined COUT_DEBUG
  std::cout << "Num poles: " << m_resultCurve->Poles().size() << std::endl;
#endif

  m_plotter.REDRAW_CURVE("faired", m_resultCurve.Access(), MobiusColor_Green);

  return true;
}
