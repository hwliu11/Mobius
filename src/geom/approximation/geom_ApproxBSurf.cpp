//-----------------------------------------------------------------------------
// Created on: 16 June 2019
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
#include <mobius/geom_ApproxBSurf.h>

// Geom includes
#include <mobius/geom_ApproxBSurfBi.h>
#include <mobius/geom_ApproxBSurfMji.h>
#include <mobius/geom_BuildAveragePlane.h>
#include <mobius/geom_PlaneSurface.h>

// BSpl includes
#include <mobius/bspl_KnotsUniform.h>

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

mobius::geom_ApproxBSurf::geom_ApproxBSurf(const t_ptr<t_pcloud>& points,
                                           core_ProgressEntry     progress,
                                           core_PlotterEntry      plotter)
: core_OPERATOR(progress, plotter)
{
  m_inputPoints = points;
  m_iDegreeU = m_iDegreeV = 0; // Will be initialized from the initial surface.
}

//-----------------------------------------------------------------------------

mobius::geom_ApproxBSurf::geom_ApproxBSurf(const t_ptr<t_pcloud>& points,
                                           const int              uDegree,
                                           const int              vDegree,
                                           core_ProgressEntry     progress,
                                           core_PlotterEntry      plotter)
: core_OPERATOR(progress, plotter)
{
  m_inputPoints = points;
  m_iDegreeU    = uDegree;
  m_iDegreeV    = vDegree;
}

//-----------------------------------------------------------------------------

void mobius::geom_ApproxBSurf::SetInitSurface(const t_ptr<t_bsurf>& initSurf)
{
  m_initSurf = initSurf;
}

//-----------------------------------------------------------------------------

bool mobius::geom_ApproxBSurf::Perform()
{
  /* ===================
   *  Preparation stage
   * =================== */

  // Initial surface.
  if ( m_initSurf.IsNull() )
  {
    m_progress.SendLogMessage(MobiusNotice(Normal) << "Initial surface is null: initializing.");

    if ( !this->initializeSurf() )
    {
      m_progress.SendLogMessage(MobiusWarn(Normal) << "Failed to initialize surface.");
      return false;
    }
  }

  // Main properties.
  const int                  numPolesU = int( m_initSurf->GetPoles().size() );
  const int                  numPolesV = int( m_initSurf->GetPoles()[0].size() );
  const int                  nPoles    = numPolesU*numPolesV;
  const int                  p         = m_initSurf->GetDegree_U();
  const int                  q         = m_initSurf->GetDegree_V();

  t_ptr<t_alloc2d> alloc = new t_alloc2d;
  //
  alloc->Allocate(3, p + 1, true); // memBlockSurf_EffectiveNDersUResult
  alloc->Allocate(3, q + 1, true); // memBlockSurf_EffectiveNDersVResult
  alloc->Allocate(2, 3,     true); // memBlockSurf_EffectiveNDersUInternal
  alloc->Allocate(2, 3,     true); // memBlockSurf_EffectiveNDersVInternal

  // Prepare twovariate basis functions.
  this->prepareNk(alloc);

  /* ====================================================
   *  Parameterize data points using the initial surface
   * ==================================================== */

  // Prepare memory arena for point inversion.
  t_ptr<t_alloc2d> piAlloc = new t_alloc2d;
  //
  const int memBlock_BSplineSurfEvalD2U      = 0;
  const int memBlock_BSplineSurfEvalD2V      = 1;
  const int memBlock_BSplineSurfEvalInternal = 2;
  //
  piAlloc->Allocate(3, p + 1, true);
  piAlloc->Allocate(3, q + 1, true);
  piAlloc->Allocate(2, 2 + 1, true);

  // Loop over the points and invert each one to the surface.
  const int nPts = m_inputPoints->GetNumberOfPoints();
  //
  for ( int k = 0; k < nPts; ++k )
  {
    const t_xyz& xyz = m_inputPoints->GetPoint(k);

    // Invert point.
    t_uv projUV;
    //
    if ( !m_initSurf->InvertPoint(xyz, projUV, 1e-6,
                                  piAlloc,
                                  memBlock_BSplineSurfEvalD2U,
                                  memBlock_BSplineSurfEvalD2V,
                                  memBlock_BSplineSurfEvalInternal) )
    {
      m_progress.SendLogMessage(MobiusErr(Normal) << "Point inversion failed for point %1." << k);
      return false;
    }

    // Set projection result as a parameter pair.
    m_UVs.push_back(projUV);
  }

  /* ================================
   *  Prepare linear system to solve
   * ================================ */

  const int dim = nPoles;

#if defined COUT_DEBUG
  std::cout << "Dimension: " << dim << std::endl;
  std::cout << "Computing matrix M..." << std::endl;
#endif

  // Initialize matrix of left-hand-side coefficients.
  int r = 0;
  Eigen::MatrixXd eigen_A_mx(dim, dim);
  for ( int i = 0; i < nPoles; ++i )
  {
    // Fill upper triangle and populate the matrix symmetrically.
    int c = r;
    for ( int j = i; j < nPoles; ++j )
    {
      geom_ApproxBSurfMji M_ji_func(j, i, m_UVs, m_Nk);

      // Compute coefficient.
      const double val = M_ji_func.Eval();
      eigen_A_mx(r, c) = eigen_A_mx(c, r) = val;
      c++;
    }
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
    geom_ApproxBSurfBi rhs(k, m_inputPoints, m_UVs, m_Nk);

    // Compute integrals.
    const double val_x = rhs.Eval(0);
    const double val_y = rhs.Eval(1);
    const double val_z = rhs.Eval(2);
    //
    eigen_B_mx(r, 0) = val_x;
    eigen_B_mx(r, 1) = val_y;
    eigen_B_mx(r, 2) = val_z;
    //
    r++;
  }

#if defined COUT_DEBUG
  std::cout << "Here is the matrix B:\n" << eigen_B_mx << std::endl;
  std::cout << "Solving linear system..." << std::endl;
#endif

  /* ==============================================
   *  Solve linear system and construct the result
   * ============================================== */

  // Solve.
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> QR(eigen_A_mx);
  Eigen::MatrixXd eigen_X_mx = QR.solve(eigen_B_mx);

#if defined COUT_DEBUG
  std::cout << "Here is the matrix X (solution):\n" << eigen_X_mx << std::endl;
  std::cout << "Constructing result..." << std::endl;
#endif

  // Prepare a copy of the surface to serve as a result.
  m_resultSurf = m_initSurf->Copy();

  // Set new coordinates for poles.
  const std::vector< std::vector<t_xyz> >& poles = m_resultSurf->GetPoles();
  //
  for ( int k = 0; k < dim; ++k )
  {
    int i, j;
    this->GetIJ(k, i, j);

    t_xyz D = t_xyz( eigen_X_mx(k, 0), eigen_X_mx(k, 1), eigen_X_mx(k, 2) );
    //
    m_resultSurf->SetPole(i, j, D);
  }

  return true;
}

//-----------------------------------------------------------------------------

void mobius::geom_ApproxBSurf::prepareNk(t_ptr<t_alloc2d> alloc)
{
  const int numPolesU = int( m_initSurf->GetPoles().size() );
  const int numPolesV = int( m_initSurf->GetPoles()[0].size() );
  const int nPoles    = numPolesU*numPolesV;

  // Initialize basis functions.
  for ( int k = 0; k < nPoles; ++k )
  {
    // Prepare evaluator for N_k(u,v).
    t_ptr<geom_BSurfNk>
      Nk = new geom_BSurfNk(m_initSurf, k, alloc);
    //
    m_Nk.push_back(Nk);
  }
}

//-----------------------------------------------------------------------------

bool mobius::geom_ApproxBSurf::initializeSurf()
{
  // Build average plane.
  t_ptr<t_plane> averagePln;
  //
  geom_BuildAveragePlane planeAlgo;
  //
  if ( !planeAlgo.Build(m_inputPoints, averagePln) )
  {
    m_progress.SendLogMessage(MobiusWarn(Normal) << "Cannot build average plane.");
    return false;
  }

  // Project point cloud to plane to determine the parametric bounds.
  averagePln->TrimByPoints(m_inputPoints);

  // Convert plane to B-surf.
  m_initSurf = averagePln->ToBSurface(m_iDegreeU ? m_iDegreeU : 3,
                                      m_iDegreeV ? m_iDegreeV : 3);

  return true;
}