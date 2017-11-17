//-----------------------------------------------------------------------------
// Created on: 02 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
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
