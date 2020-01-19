//-----------------------------------------------------------------------------
// Created on: 10 March 2015
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
#include <mobius/geom_GordonSurface.h>

//! Constructs Gordon surface on the passed network of curves.
//! \param U_curves [in] iso U curves.
//! \param V_curves [in] iso V curves.
//! \param grid     [in] two-dimensional grid of joints.
mobius::geom_GordonSurface::geom_GordonSurface(const std::vector< t_ptr<t_curve> >& U_curves,
                                               const std::vector< t_ptr<t_curve> >& V_curves,
                                               const TGrid&                         grid)
{
  m_UCurves = U_curves;
  m_VCurves = V_curves;
  m_grid    = grid;
}

//! Destructor.
mobius::geom_GordonSurface::~geom_GordonSurface()
{}

//! Dumps the surface data to string stream.
//! \param stream [in/out] target stream.
void mobius::geom_GordonSurface::Dump(std::ostream* out) const
{
  *out << "Gordon surface\n";
}

//! Calculates boundary box for the Gordon surface.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::geom_GordonSurface::GetBounds(double& xMin, double& xMax,
                                           double& yMin, double& yMax,
                                           double& zMin, double& zMax) const
{
  xMin = yMin = zMin =  DBL_MAX;
  xMax = yMax = zMax = -DBL_MAX;

  // U isos
  for ( size_t i = 0; i < m_UCurves.size(); ++i )
  {
    double xMin_crv, xMax_crv, yMin_crv, yMax_crv, zMin_crv, zMax_crv;
    m_UCurves[i]->GetBounds(xMin_crv, xMax_crv, yMin_crv, yMax_crv, zMin_crv, zMax_crv);

    xMin = std::min(xMin, xMin_crv);
    yMin = std::min(yMin, yMin_crv);
    zMin = std::min(zMin, zMin_crv);
    xMax = std::max(xMax, xMax_crv);
    yMax = std::max(yMax, yMax_crv);
    zMax = std::max(zMax, zMax_crv);
  }

  // V isos
  for ( size_t i = 0; i < m_VCurves.size(); ++i )
  {
    double xMin_crv, xMax_crv, yMin_crv, yMax_crv, zMin_crv, zMax_crv;
    m_VCurves[i]->GetBounds(xMin_crv, xMax_crv, yMin_crv, yMax_crv, zMin_crv, zMax_crv);

    xMin = std::min(xMin, xMin_crv);
    yMin = std::min(yMin, yMin_crv);
    zMin = std::min(zMin, zMin_crv);
    xMax = std::max(xMax, xMax_crv);
    yMax = std::max(yMax, yMax_crv);
    zMax = std::max(zMax, zMax_crv);
  }
}

//! Returns first parameter in U dimension.
//! \return first parameter.
double mobius::geom_GordonSurface::GetMinParameter_U() const
{
  return 0.0;//m_pU[0];
}

//! Returns last parameter in U dimension.
//! \return last parameter.
double mobius::geom_GordonSurface::GetMaxParameter_U() const
{
  return 0.0;//m_pU[m_iU-1];
}

//! Returns first parameter in V dimension.
//! \return first parameter.
double mobius::geom_GordonSurface::GetMinParameter_V() const
{
  return 0.0;//m_pV[0];
}

//! Returns last parameter in V dimension.
//! \return last parameter.
double mobius::geom_GordonSurface::GetMaxParameter_V() const
{
  return 0.0;//m_pV[m_iV-1];
}

//! Evaluates Gordon surface for the given pair of (u, v) parameters.
//! \param u [in] U parameter value to evaluate surface for.
//! \param v [in] V parameter value to evaluate surface for.
//! \param C [out] 3D point corresponding to the given parameter pair.
void mobius::geom_GordonSurface::Eval(const double u,
                                      const double v,
                                      t_xyz&       C) const
{
  /*const int n = this->get_n();
  const int m = this->get_m();
  std::vector<double> u_knots = this->u_knots();
  std::vector<double> v_knots = this->v_knots();

  xyz SN;
  for ( int i = 0; i <= n; ++i )
  {
    const ptr<curve>& cu = m_UCurves[i];
    xyz cu_P;
    cu->Eval(u, cu_P);

    SN += cu_P*L_ks(i, n, v, v_knots);
  }

  xyz SM;
  for ( int j = 0; j <= m; ++j )
  {
    const ptr<curve>& bv = m_VCurves[j];
    xyz bv_P;
    bv->Eval(v, bv_P);

    SM += bv_P*L_ks(j, m, u, u_knots);
  }

  xyz SSMN;
  for ( int j = 0; j <= m; ++j )
  {
    for ( int i = 0; i <= n; ++i )
    {
      const TGridPointList& gridLine_i = m_grid[i + 1];
      const xyz& p_ij = gridLine_i[j + 1].P;

      SSMN += p_ij*L_ks(j, m, u, u_knots)*L_ks(i, n, v, v_knots);
    }
  }

  C = SN + SM - SSMN;*/
}

//std::vector<double> mobius::geom_GordonSurface::u_knots() const
//{
//  std::vector<double> aResult;
//  const TGridPointList& aGridLine = m_Pts(1);
//  for ( int i = 1; i <= aGridLine.Length(); ++i )
//  {
//    double u = aGridLine(i).U;
//    aResult.Append(u);
//  }
//  return aResult;
//}
//
//std::vector<double> mobius::geom_GordonSurface::v_knots() const
//{
//  std::vector<double> aResult;
//  for ( int i = 1; i <= m_Pts.Length(); ++i )
//  {
//    const TGridPointList& aGridLine = m_Pts(i);
//    double v = aGridLine(1).V;
//    aResult.Append(v);
//  }
//  return aResult;
//}

double mobius::geom_GordonSurface::L_ks(const int k,
                                        const int s,
                                        const double t,
                                        const std::vector<double>& t_knots) const
{
  double A = A_ks(k, s, t, t_knots);
  double B = B_ks(k, s, t, t_knots);
  const double L = A / B;
  return L;
}

double mobius::geom_GordonSurface::DL_ks(const int k,
                                         const int s,
                                         const double t,
                                         const std::vector<double>& t_knots) const
{
  double DA = DA_ks(k, s, t, t_knots);
  double B = B_ks(k, s, t, t_knots);
  const double DL = DA / B;
  return DL;
}

double mobius::geom_GordonSurface::A_ks(const int k,
                                        const int s,
                                        const double t,
                                        const std::vector<double>& t_knots) const
{
  double A = 1.0;
  for ( int j = 0; j <= s; ++j )
  {
    if ( j == k )
      continue;

    const double t_knot_j = t_knots[j];
    A *= (t - t_knot_j);
  }
  return A;
}

double mobius::geom_GordonSurface::B_ks(const int k,
                                        const int s,
                                        const double t,
                                        const std::vector<double>& t_knots) const
{
  double B = 1.0;
  const double t_knot_k = t_knots[k];
  for ( int j = 0; j <= s; ++j )
  {
    if ( j == k )
      continue;

    const double t_knot_j = t_knots[j];
    B *= (t_knot_k - t_knot_j);
  }
  return B;
}

double mobius::geom_GordonSurface::DA_ks(const int k,
                                         const int s,
                                         const double t,
                                         const std::vector<double>& t_knots) const
{
  /*double D = 1.0;
  for ( int i = 0; i <= s; ++i )
  {
    if ( i == k )
      continue;

    v DD = 1.0;
    for ( int j = 0; j <= s; ++j )
    {
      if ( j == k || j == i )
        continue;

      const double t_knot_j = t_knots[j];
      DD *= (t - t_knot_j);
    }

    D += DD;
  }
  return D;*/
  return 0.0;
}
