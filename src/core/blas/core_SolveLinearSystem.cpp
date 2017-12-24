//-----------------------------------------------------------------------------
// Created on: 02 November 2013
//-----------------------------------------------------------------------------
// Copyright (c) 2017, Sergey Slyadnev
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
#include <mobius/core_SolveLinearSystem.h>

// Eigen includes
#include <Eigen/Dense>

#undef COUT_DEBUG
#if defined COUT_DEBUG
  #pragma message("===== warning: COUT_DEBUG is enabled")
#endif

//! Constructor.
mobius::core_SolveLinearSystem::core_SolveLinearSystem()
{}

//! Destructor.
mobius::core_SolveLinearSystem::~core_SolveLinearSystem()
{}

//! Solves linear system.
//! \param A   [in]  square matrix of coefficients.
//! \param b   [in]  right-hand side.
//! \param x   [out] solution vector.
//! \param dim [in]  dimension.
void mobius::core_SolveLinearSystem::operator()(const double* A,
                                                const double* b,
                                                double*       x,
                                                const int     dim)
{
  // Initialize matrix from the passed row pointer
  Eigen::MatrixXd eigen_A(dim, dim);
  for ( int r = 0; r < dim; ++r )
    for ( int c = 0; c < dim; ++c )
      eigen_A(r, c) = A[r*dim + c];

  // Initialize vector of right hand side
  Eigen::VectorXd eigen_b(dim);
  for ( int r = 0; r < dim; ++r )
    eigen_b(r) = b[r];

  // Solve
  Eigen::VectorXd eigen_x = eigen_A.colPivHouseholderQr().solve(eigen_b);

#if defined COUT_DEBUG
  std::cout << "Here is the matrix A:\n" << eigen_A << std::endl;
  std::cout << "Here is the vector b:\n" << eigen_b << std::endl;
  std::cout << "The solution is:\n" << eigen_x << std::endl;
#endif

  // Pack results
  for ( int r = 0; r < dim; ++r )
    x[r] = eigen_x(r);
}
