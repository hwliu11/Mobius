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
#include <mobius/geom_ApproxBSurfMij.h>
#include <mobius/geom_BuildAveragePlane.h>
#include <mobius/geom_FairBSurfAkl.h>
#include <mobius/geom_PlaneSurface.h>

// Eigen includes
#pragma warning(push, 0)
#pragma warning(disable : 4702 4701)
#include <Eigen/Dense>
#pragma warning(default : 4702 4701)
#pragma warning(pop)

// Standard includes
#include <map>

//-----------------------------------------------------------------------------

#undef COUT_DEBUG
#if defined COUT_DEBUG
  #pragma message("===== warning: COUT_DEBUG is enabled")
#endif

//-----------------------------------------------------------------------------

mobius::geom_ApproxBSurf::geom_ApproxBSurf(const t_ptr<t_pcloud>& points,
                                           const t_ptr<t_bsurf>&  initSurf,
                                           core_ProgressEntry     progress,
                                           core_PlotterEntry      plotter)
: geom_OptimizeBSurfBase(initSurf, progress, plotter)
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
: geom_OptimizeBSurfBase(progress, plotter)
{
  m_inputPoints = points;
  m_iDegreeU    = uDegree;
  m_iDegreeV    = vDegree;
}

//-----------------------------------------------------------------------------

bool mobius::geom_ApproxBSurf::Perform(const double lambda)
{
  m_progress.Init();

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
  const int                  p         = m_initSurf->GetDegree_U();
  const int                  q         = m_initSurf->GetDegree_V();
  const std::vector<double>& U         = m_initSurf->GetKnots_U();
  const std::vector<double>& V         = m_initSurf->GetKnots_V();
  const int                  numPolesU = int( m_initSurf->GetPoles().size() );
  const int                  numPolesV = int( m_initSurf->GetPoles()[0].size() );
  const int                  nPoles    = numPolesU*numPolesV;
  const int                  dim       = nPoles;

  t_ptr<t_alloc2d> alloc = new t_alloc2d;
  //
  alloc->Allocate(3, p + 1, true); // memBlockSurf_EffectiveNDersUResult
  alloc->Allocate(3, q + 1, true); // memBlockSurf_EffectiveNDersVResult
  alloc->Allocate(2, 3,     true); // memBlockSurf_EffectiveNDersUInternal
  alloc->Allocate(2, 3,     true); // memBlockSurf_EffectiveNDersVInternal

  // Prepare twovariate basis functions.
  if ( !this->prepareNk(alloc) )
  {
    m_progress.SendLogMessage(MobiusErr(Normal) << "Failed to prepare basis N_k functions.");
    return false;
  }

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

  std::cout << "Dimension: " << dim << std::endl;

#if defined COUT_DEBUG
  std::cout << "Computing matrix M..." << std::endl;
#endif

  m_progress.Init(nPoles);

  // Initialize matrix of left-hand-side coefficients.
  int r = 0;
  Eigen::MatrixXd eigen_M1_mx(dim, dim);
  for ( int i = 0; i < nPoles; ++i )
  {
    // Fill upper triangle and populate the matrix symmetrically.
    int c = r;
    for ( int j = i; j < nPoles; ++j )
    {
      geom_ApproxBSurfMij M_ij_func(i, j, m_UVs, m_Nk);

      // Compute coefficient.
      const double val = M_ij_func.Eval();
      eigen_M1_mx(r, c) = eigen_M1_mx(c, r) = val;
      c++;
    }
    r++;

    m_progress.StepProgress(1);

    if ( m_progress.IsCancelling() )
      return false;

#if defined COUT_DEBUG
    std::cout << "M " << r << " done" << std::endl;
#endif
  }
  //
  m_progress.StepProgress(1);

  Eigen::MatrixXd eigen_M_mx(dim, dim);
  //
  eigen_M_mx = eigen_M1_mx;

  if ( m_progress.IsCancelling() )
    return false;

  // Add fairing terms.
  if ( lambda > 0 )
  {
    // Initialize matrix of fairing coefficients.
    r = 0;
    Eigen::MatrixXd eigen_M2_mx(dim, dim);
    for ( int i = 0; i < nPoles; ++i )
    {
      // Fill upper triangle and populate the matrix symmetrically.
      int c = r;
      for ( int j = i; j < nPoles; ++j )
      {
        geom_FairBSurfAkl A_ij_func(i, j, lambda, m_Nk, true);

        // Compute integral.
        const double val = A_ij_func.Integral(U, V, p, q);
        eigen_M2_mx(r, c) = eigen_M2_mx(c, r) = val;
        c++;
      }
      r++;

#if defined COUT_DEBUG
      std::cout << "A " << r << " done" << std::endl;
#endif
    }

    // Sum two matrices.
    eigen_M_mx += eigen_M2_mx;
  }

  if ( m_progress.IsCancelling() )
    return false;

  std::cout << "\t>>> det(M) = " << eigen_M_mx.determinant() << std::endl;

#if defined COUT_DEBUG
  std::cout << "Here is the matrix M:\n" << eigen_M_mx << std::endl;
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

  if ( m_progress.IsCancelling() )
    return false;

  // Solve.
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> QR(eigen_M_mx);
  Eigen::MatrixXd eigen_X_mx = QR.solve(eigen_B_mx);

#if defined COUT_DEBUG
  std::cout << "Here is the matrix X (solution):\n" << eigen_X_mx << std::endl;
  std::cout << "Constructing result..." << std::endl;
#endif

  // Prepare a copy of the surface to serve as a result.
  m_resultSurf = m_initSurf->Copy();

  // Set new coordinates for poles.
  for ( int rr = 0; rr < dim; ++rr )
  {
    int i, j;
    this->GetIJ(rr, i, j);

    if ( !this->IsPinned(rr) )
    {
      t_xyz D = t_xyz( eigen_X_mx(rr, 0), eigen_X_mx(rr, 1), eigen_X_mx(rr, 2) );
      //
      m_resultSurf->SetPole(i, j, D);
    }
  }

  return true;
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
  averagePln->TrimByPoints(m_inputPoints, 10);

  // Convert plane to B-surf.
  t_ptr<t_bsurf> initSurf = averagePln->ToBSurface(m_iDegreeU ? m_iDegreeU : 3,
                                                   m_iDegreeV ? m_iDegreeV : 3);

  // Initialize surface.
  this->SetInitSurface(initSurf);

  return true;
}
