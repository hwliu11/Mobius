//-----------------------------------------------------------------------------
// Created on: 07 February 2014
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
#include <mobius/geom_BSplineSurface.h>

// Core includes
#include <mobius/core_HeapAlloc.h>

// BSpl includes
#include <mobius/bspl_EffectiveN.h>
#include <mobius/bspl_FindSpan.h>

//! Constructor.
//! \param Poles [in] poles for B-spline surface.
//! \param U     [in] knot vector in U dimension.
//! \param V     [in] knot vector in V dimension.
//! \param nU    [in] number of knots in U dimension.
//! \param nV    [in] number of knots in V dimension.
//! \param p     [in] degree in U dimension.
//! \param q     [in] degree in V dimension.
mobius::geom_BSplineSurface::geom_BSplineSurface(const std::vector< std::vector<xyz> >& Poles,
                                                 const double*                          U,
                                                 const double*                          V,
                                                 const int                              nU,
                                                 const int                              nV,
                                                 const int                              p,
                                                 const int                              q)
: geom_Surface()
{
  std::vector<double> Uvec;
  for ( size_t i = 0; i < nU; ++i )
    Uvec.push_back(U[i]);

  std::vector<double> Vvec;
  for ( size_t i = 0; i < nV; ++i )
    Vvec.push_back(V[i]);

  this->init(Poles, Uvec, Vvec, p, q);
}

//! Constructor.
//! \param Poles [in] poles for B-spline surface.
//! \param U     [in] knot vector in U dimension.
//! \param V     [in] knot vector in V dimension.
//! \param p     [in] degree in U dimension.
//! \param q     [in] degree in V dimension.
mobius::geom_BSplineSurface::geom_BSplineSurface(const std::vector< std::vector<xyz> >& Poles,
                                                 const std::vector<double>&             U,
                                                 const std::vector<double>&             V,
                                                 const int                              p,
                                                 const int                              q)
: geom_Surface()
{
  this->init(Poles, U, V, p, q);
}

//! Destructor.
mobius::geom_BSplineSurface::~geom_BSplineSurface()
{}

//! Dumps the surface data to string stream.
//! \param stream [in/out] target stream.
void mobius::geom_BSplineSurface::Dump(std::stringstream& stream) const
{
  stream << "B-surface with the following properties:\n"
         << "\t U degree (p) = " << m_iDegU << "\n"
         << "\t V degree (q) = " << m_iDegV << "\n";
}

//! Calculates boundary box for the B-spline surface by its control polygon.
//! Notice that this peculiarity can look weird as control polygon only
//! outlines the B-spline surface, but does not follow its exact shape.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::geom_BSplineSurface::Bounds(double& xMin, double& xMax,
                                         double& yMin, double& yMax,
                                         double& zMin, double& zMax) const
{
  double x_min = DBL_MAX, x_max = -DBL_MAX;
  double y_min = DBL_MAX, y_max = -DBL_MAX;
  double z_min = DBL_MAX, z_max = -DBL_MAX;

  // B-spline surface is fully contained in its control grid, so we can take
  // it as a rough solution
  for ( int i = 0; i < (int) m_poles.size(); ++i )
  {
    const std::vector<xyz>& uLine = m_poles.at(i);
    for ( int j = 0; j < (int) uLine.size(); ++j )
    {
      const xyz& P = uLine.at(j);
      const double x = P.X(), y = P.Y(), z = P.Z();

      if ( x > x_max )
        x_max = x;
      if ( x < x_min )
        x_min = x;
      if ( y > y_max )
        y_max = y;
      if ( y < y_min )
        y_min = y;
      if ( z > z_max )
        z_max = z;
      if ( z < z_min )
        z_min = z;
    }
  }

  // Set results
  xMin = x_min;
  xMax = x_max;
  yMin = y_min;
  yMax = y_max;
  zMin = z_min;
  zMax = z_max;
}

//! Returns first knot in U dimension.
//! \return first knot.
double mobius::geom_BSplineSurface::MinParameter_U() const
{
  return m_U[0];
}

//! Returns last knot in U dimension.
//! \return last knot.
double mobius::geom_BSplineSurface::MaxParameter_U() const
{
  return m_U[m_U.size()-1];
}

//! Returns first knot in V dimension.
//! \return first knot.
double mobius::geom_BSplineSurface::MinParameter_V() const
{
  return m_V[0];
}

//! Returns last knot in V dimension.
//! \return last knot.
double mobius::geom_BSplineSurface::MaxParameter_V() const
{
  return m_V[m_V.size()-1];
}

//! Evaluates B-spline surface for the given pair of (u, v) parameters.
//! \param u [in]  U parameter value to evaluate surface for.
//! \param v [in]  V parameter value to evaluate surface for.
//! \param C [out] 3D point corresponding to the given parameter pair.
void mobius::geom_BSplineSurface::Eval(const double u,
                                       const double v,
                                       xyz&         C) const
{
  // Heap allocator
  core_HeapAlloc<double> Alloc;

  // Find span the passed u and v fall into
  bspl_FindSpan FindSpanU(m_U, m_iDegU);
  bspl_FindSpan FindSpanV(m_V, m_iDegV);
  //
  const int span_u = FindSpanU(u);
  const int span_v = FindSpanV(v);

  //---------------------------------------------
  // Evaluate effective B-spline basis functions
  //---------------------------------------------

  bspl_EffectiveN EffectiveN;
  double* N_u = Alloc.Allocate(m_iDegU + 1, true);
  double* N_v = Alloc.Allocate(m_iDegV + 1, true);
  EffectiveN(u, m_U, m_iDegU, span_u, N_u);
  EffectiveN(v, m_V, m_iDegV, span_v, N_v);

  const int u_first_idx = span_u - m_iDegU;
  const int v_first_idx = span_v - m_iDegV;

  //---------------------------
  // Evaluate B-spline surface
  //---------------------------

  xyz Res;
  for ( int i = 0; i <= m_iDegU; ++i )
  {
    xyz temp;
    for ( int j = 0; j <= m_iDegV; ++j )
    {
      const xyz& P_ij = m_poles.at(u_first_idx + i).at(v_first_idx + j);
      temp += P_ij*N_v[j];
    }
    Res += temp*N_u[i];
  }

  // Set output parameter
  C = Res;
}

//! Extracts isoparametric curve corresponding to the passed {u} level.
//! \param u [in] parameter value to extract isoparametric curve for.
//! \return isoline.
mobius::Ptr<mobius::bcurve>
  mobius::geom_BSplineSurface::Iso_U(const double u) const
{
  // Heap allocator
  core_HeapAlloc<double> Alloc;

  // Find span the passed u falls into
  bspl_FindSpan FindSpan(m_U, m_iDegU);
  const int span_u = FindSpan(u);

  //--------------------------------------------------
  // Calculate control points for isoparametric curve
  //--------------------------------------------------

  // Evaluate effective B-spline basis functions
  bspl_EffectiveN EffectiveN;
  double* N_u = Alloc.Allocate(m_iDegU + 1, true);
  EffectiveN(u, m_U, m_iDegU, span_u, N_u);
  const int u_first_idx = span_u - m_iDegU;

  // Calculate new poles
  std::vector<xyz> Q;
  for ( int j = 0; j < (int) m_poles[0].size(); ++j )
  {
    xyz Q_j;
    for ( int i = 0; i <= m_iDegU; ++i )
    {
      const xyz& P_ij = m_poles[u_first_idx + i][j];
      Q_j += P_ij*N_u[i];
    }
    Q.push_back(Q_j);
  }

  //-----------------------
  // Create B-spline curve
  //-----------------------

  Ptr<bcurve> Iso = new bcurve(Q, m_V, m_iDegV);
  return Iso;
}

//! Extracts isoparametric curve corresponding to the passed {v} level.
//! \param v [in] parameter value to extract isoparametric curve for.
//! \return isoline.
mobius::Ptr<mobius::bcurve>
  mobius::geom_BSplineSurface::Iso_V(const double v) const
{
  // Heap allocator
  core_HeapAlloc<double> Alloc;

  // Find span the passed u falls into
  bspl_FindSpan FindSpan(m_V, m_iDegV);
  const int span_v = FindSpan(v);

  //--------------------------------------------------
  // Calculate control points for isoparametric curve
  //--------------------------------------------------

  // Evaluate effective B-spline basis functions
  bspl_EffectiveN EffectiveN;
  double* N_v = Alloc.Allocate(m_iDegV + 1, true);
  EffectiveN(v, m_V, m_iDegV, span_v, N_v);
  const int v_first_idx = span_v - m_iDegV;

  // Calculate new poles
  std::vector<xyz> Q;
  for ( int i = 0; i < (int) m_poles.size(); ++i )
  {
    xyz Q_i;
    for ( int j = 0; j <= m_iDegV; ++j )
    {
      const xyz& P_ij = m_poles[i][v_first_idx + j];
      Q_i += P_ij*N_v[j];
    }
    Q.push_back(Q_i);
  }

  //-----------------------
  // Create B-spline curve
  //-----------------------

  Ptr<bcurve> Iso = new bcurve(Q, m_U, m_iDegU);
  return Iso;
}

//! Initializes B-spline surface with complete data.
//! \param Poles [in] control points.
//! \param U     [in] knot vector in U dimension.
//! \param V     [in] knot vector in V dimension.
//! \param p     [in] degree of the B-spline basis functions in U dimension.
//! \param q     [in] degree of the B-spline basis functions in V dimension.
void mobius::geom_BSplineSurface::init(const std::vector< std::vector<xyz> >& Poles,
                                       const std::vector<double>&             U,
                                       const std::vector<double>&             V,
                                       const int                              p,
                                       const int                              q)
{
  m_poles = Poles;
  m_U     = U;
  m_V     = V;
  m_iDegU = p;
  m_iDegV = q;
}
