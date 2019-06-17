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
}

//-----------------------------------------------------------------------------

void mobius::geom_ApproxBSurf::InitSurface(const t_ptr<t_bsurf>& initSurf)
{
  m_initSurf = initSurf;
}

//-----------------------------------------------------------------------------

bool mobius::geom_ApproxBSurf::Perform()
{
  /* =================
   *  Contract checks
   * ================= */

  if ( m_initSurf.IsNull() )
  {
    m_progress.SendLogMessage(MobiusErr(Normal) << "Initial surface is null.");
    return false;
  }

  /* ===================
   *  Preparation stage
   * =================== */

  // Main properties.
  const int                  numPolesU = int( m_initSurf->GetPoles().size() );
  const int                  numPolesV = int( m_initSurf->GetPoles()[0].size() );
  const int                  nPoles    = numPolesU*numPolesV;
  const std::vector<double>& U         = m_initSurf->GetKnots_U();
  const std::vector<double>& V         = m_initSurf->GetKnots_V();
  const int                  p         = m_initSurf->GetDegree_U();
  const int                  q         = m_initSurf->GetDegree_V();

  t_ptr<t_alloc2d> alloc = new t_alloc2d;

  // Prepare twovariate basis functions.
  this->prepareNk(alloc);

  /* ====================================================
   *  Parameterize data points using the initial surface
   * ==================================================== */

  // Prepare memory arena for point imversion.
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

  // TODO: NYI

  /* ==============================================
   *  Solve linear system and construct the result
   * ============================================== */

  return false;
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
