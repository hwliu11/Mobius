//-----------------------------------------------------------------------------
// Created on: 03 October 2018
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
#include <mobius/geom_InterpolateMultiCurve.h>

// Geom includes
#include <mobius/geom_InterpolateCurve.h>

// BSpl includes
#include <mobius/bspl_KnotsAverage.h>
#include <mobius/bspl_ParamsCentripetal.h>
#include <mobius/bspl_ParamsChordLength.h>

//-----------------------------------------------------------------------------

mobius::geom_InterpolateMultiCurve::geom_InterpolateMultiCurve(const int                  deg,
                                                               const bspl_ParamsSelection paramsType,
                                                               const bspl_KnotsSelection  knotsType,
                                                               core_ProgressEntry         progress,
                                                               core_PlotterEntry          plotter)
: core_OPERATOR(progress, plotter)
{
  m_iDeg       = deg;
  m_paramsType = paramsType;
  m_knotsType  = knotsType;
}

//-----------------------------------------------------------------------------

void mobius::geom_InterpolateMultiCurve::AddRow(const std::vector<t_xyz>& points)
{
  m_pointsGrid.push_back(points);
}

//-----------------------------------------------------------------------------

bool mobius::geom_InterpolateMultiCurve::Perform()
{
  /* =====================================
   *  Choose unified parameters and knots
   * ===================================== */

  const int numSections = int( m_pointsGrid.size() );

  // Heap allocator.
  mobius::core_HeapAlloc<double> Alloc;

  // We use the requested parameterization technique for each curve. Then
  // the idea is simply to average the parameters for each point by the number
  // of curves. In the same loop, interpolation knots are selected. The same
  // averaging technique is applied to these knots then. The following loop
  // is to compute parameters and knots for each section independently.
  std::vector<double*> paramsForSections, knotsForSections;
  //
  int common_n = 0, common_m = 0;
  //
  for ( int secIdx = 0; secIdx < numSections; ++secIdx )
  {
    const std::vector<t_xyz>& points = m_pointsGrid[secIdx];

    // Choose interpolation parameters. There are as many parameters as
    // many data points are passed for interpolation.
    const int n = (int) (points.size() - 1);
    //
    if ( secIdx == 0 )
    {
      common_n = n;
    }
    else if ( n != common_n )
    {
      m_progress.SendLogMessage(MobiusErr(Normal) << "Number of points in "
                                                     "section %1 is different "
                                                     "from the expected number %2."
                                                  << secIdx << common_n);
      return false;
    }
    //
    double* params = Alloc.Allocate(n + 1, true);

    // Choose parameterization technique.
    if ( m_paramsType == ParamsSelection_Centripetal )
    {
      if ( bspl_ParamsCentripetal::Calculate(points, params) != bspl_ParamsCentripetal::ErrCode_NoError )
      {
        m_progress.SendLogMessage(MobiusErr(Normal) << "Cannot compute centripetal parameters "
                                                       "for section %1 (centripetal method)."
                                                    << secIdx);
        return false;
      }
    }
    else if ( m_paramsType == ParamsSelection_ChordLength )
    {
      if ( bspl_ParamsChordLength::Calculate(points, params) != bspl_ParamsChordLength::ErrCode_NoError )
      {
        m_progress.SendLogMessage(MobiusErr(Normal) << "Cannot compute chord-length parameters "
                                                       "for section %1 (centripetal method)."
                                                    << secIdx);
        return false;
      }
    }
    else
    {
      m_progress.SendLogMessage(MobiusErr(Normal) << "Unsupported parameterization type.");
      return false;
    }
    //
    paramsForSections.push_back(params);

    // Choose the last knot index. In this particular case, the index is
    // chosen for an interpolation scheme without end-point constraints.
    const int m = bspl::M(n, m_iDeg);
    double*   U = Alloc.Allocate(m + 1, true);
    //
    if ( secIdx == 0 )
    {
      common_m = m;
    }
    //
    if ( m_knotsType == KnotsSelection_Average )
    {
      if ( bspl_KnotsAverage::Calculate(params,
                                        n,
                                        m_iDeg,
                                        m,
                                        bspl_KnotsAverage::Recognize(false, false, false, false),
                                        U) != bspl_KnotsAverage::ErrCode_NoError )
      {
        m_progress.SendLogMessage(MobiusErr(Normal) << "Cannot compute average knots "
                                                       "for section %1."
                                                    << secIdx);
        return false;
      }
    }
    else if ( m_knotsType == KnotsSelection_Uniform )
    {
      m_progress.SendLogMessage(MobiusErr(Normal) << "Unsupported knots selection strategy.");
      return false;
    }
    //
    knotsForSections.push_back(U);
  }

  // Average parameters.
  double* unifiedParams = Alloc.Allocate(common_n + 1, true);
  //
  for ( int k = 0; k < common_n + 1; ++k )
  {
    for ( int secIdx = 0; secIdx < numSections; ++secIdx )
    {
      unifiedParams[k] += paramsForSections[secIdx][k];
    }
    unifiedParams[k] /= numSections;
  }

  // Average knots.
  double* unifiedKnots = Alloc.Allocate(common_m + 1, true);
  //
  for ( int k = 0; k < common_m + 1; ++k )
  {
    for ( int secIdx = 0; secIdx < numSections; ++secIdx )
    {
      unifiedKnots[k] += knotsForSections[secIdx][k];
    }
    unifiedKnots[k] /= numSections;
  }

  /* ==========================================================
   *  Interpolate each curve passing the same parameters/knots
   * ========================================================== */

  for ( int secIdx = 0; secIdx < numSections; ++secIdx )
  {
    const std::vector<t_xyz>& points = m_pointsGrid[secIdx];

    // Iterpolate curve given that each interpolation task in a loop is
    // solved on the same parameters and knots vectors.
    geom_InterpolateCurve tool(points, m_iDeg, unifiedParams, common_n, unifiedKnots, common_m);
    //
    if ( !tool.Perform() )
    {
      m_progress.SendLogMessage( MobiusErr(Normal) << "Interpolation failed with error code %1 on section %2."
                                                   << tool.GetErrorCode() << secIdx );
      return false;
    }

    // Add interpolant to the result.
    m_curves.push_back( tool.GetResult() );
  }

  return true;
}
