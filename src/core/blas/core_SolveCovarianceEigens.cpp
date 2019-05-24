//-----------------------------------------------------------------------------
// Created on: 23 May 2019
//-----------------------------------------------------------------------------
// Copyright (c) 2019-present, Sergey Slyadnev
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
#include <mobius/core_SolveCovarianceEigens.h>

// Core includes
#include <mobius/core_Axis.h>
#include <mobius/core_Precision.h>

// Eigen includes
#pragma warning(disable : 4702 4701)
#include <Eigen/Dense>
#pragma warning(default : 4702 4701)

#define COUT_DEBUG
#if defined COUT_DEBUG
  #pragma message("===== warning: COUT_DEBUG is enabled")
#endif

namespace mobius {
namespace covariance {

bool compare(const std::pair<double, int>& p1, const std::pair<double, int>& p2)
{
  return p1.first > p2.first;
}

};
};

//-----------------------------------------------------------------------------

mobius::core_SolveCovarianceEigens::core_SolveCovarianceEigens()
{}

//-----------------------------------------------------------------------------

mobius::core_SolveCovarianceEigens::~core_SolveCovarianceEigens()
{}

//-----------------------------------------------------------------------------

bool mobius::core_SolveCovarianceEigens::operator()(const std::vector<xyz>& pts,
                                                    xyz&                    center,
                                                    xyz&                    Dx,
                                                    xyz&                    Dy,
                                                    xyz&                    Dz)
{
  const size_t nPts = pts.size();

  /* ======================
   *  Calculate mean point
   * ====================== */

  xyz mu;
  for ( size_t i = 0; i < nPts; ++i )
  {
    mu += pts[i];
  }
  mu /= int(nPts);

  /* =========================
   *  Build covariance matrix
   * ========================= */

  Eigen::Matrix3d C;
  for ( int j = 0; j <= 2; ++j )
  {
    for ( int k = 0; k <= 2; ++k )
    {
      C(j, k) = 0.0; // TODO: is that necessary?
    }
  }

  for ( int i = 0; i < nPts; ++i )
  {
    const xyz& p      = pts[i];
    xyz        p_dash = p - mu;

    for ( int j = 0; j <= 2; ++j )
    {
      for ( int k = 0; k <= 2; ++k )
      {
        C(j, k) += ( p_dash.Coord(j)*p_dash.Coord(k) );
      }
    }
  }

  for ( int j = 0; j <= 2; ++j )
  {
    for ( int k = 0; k <= 2; ++k )
    {
      C(j, k) /= nPts;
    }
  }

  Eigen::EigenSolver<Eigen::Matrix3d> EigenSolver(C);

#if defined COUT_DEBUG
  std::cout << "\t[Mobius] Covariance matrix: " << std::endl << C << std::endl;
  std::cout << "\t[Mobius] The eigen values of C are:" << std::endl << EigenSolver.eigenvalues() << std::endl;
  std::cout << "\t[Mobius] The matrix of eigenvectors, V, is:" << std::endl << EigenSolver.eigenvectors() << std::endl << std::endl;
#endif

  Eigen::Vector3cd v1 = EigenSolver.eigenvectors().col(0);
  Eigen::Vector3cd v2 = EigenSolver.eigenvectors().col(1);
  Eigen::Vector3cd v3 = EigenSolver.eigenvectors().col(2);

  xyz V[3] = { xyz( v1.x().real(), v1.y().real(), v1.z().real() ),
               xyz( v2.x().real(), v2.y().real(), v2.z().real() ),
               xyz( v3.x().real(), v3.y().real(), v3.z().real() ) };
  //
  std::vector< std::pair<double, int> >
    lambda { std::pair<double, int>( EigenSolver.eigenvalues()(0).real(), 0 ),
             std::pair<double, int>( EigenSolver.eigenvalues()(1).real(), 1 ),
             std::pair<double, int>( EigenSolver.eigenvalues()(2).real(), 2 ) };
  //
  std::sort(lambda.begin(), lambda.end(), covariance::compare);
  //
  xyz vec_X( V[lambda[0].second] );
  xyz vec_Y( V[lambda[1].second] );
  xyz vec_Z( V[lambda[2].second] );
  //
  if ( (vec_X ^ vec_Y).Modulus() < core_Precision::Resolution3D() ||
       (vec_X ^ vec_Z).Modulus() < core_Precision::Resolution3D() ||
       (vec_Y ^ vec_Z).Modulus() < core_Precision::Resolution3D() )
  {
#if defined COUT_DEBUG
    std::cout << "Warning: degenerated normal" << std::endl;
#endif
    return false; // Degenerated normal
  }

  // Check if the system is right-handed
  const double ang = vec_X.AngleWithRef(vec_Y, vec_Z);
  //
  if ( ang < 0 ) // Flip
  {
    xyz tmp = vec_X;
    vec_X = vec_Y;
    vec_Y = tmp;
  }

  // Store results
  center = mu;
  Dx     = vec_X;
  Dy     = vec_Y;
  Dz     = vec_Z;
  return true;
}
