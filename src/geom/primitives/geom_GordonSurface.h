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

#ifndef geom_GordonSurface_HeaderFile
#define geom_GordonSurface_HeaderFile

// Geometry includes
#include <mobius/geom_Curve.h>
#include <mobius/geom_Surface.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Gordon surface.
//!
//! \todo this primitive is currently not operational and deserves reconsidering.
class geom_GordonSurface : public geom_Surface
{
public:

  struct TGridPoint
  {
    double U;
    double V;
    xyz    P;

    TGridPoint() : U(0.0), V(0.0) {}
    TGridPoint(const double _U,
               const double _V,
               const xyz&   _P) : U(_U), V(_V), P(_P) {}
  };

  typedef std::vector<TGridPoint>     TGridPointList;
  typedef std::vector<TGridPointList> TGrid;

// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_GordonSurface(const std::vector< ptr<curve> >& U_curves,
                       const std::vector< ptr<curve> >& V_curves,
                       const TGrid&                     grid);

  mobiusGeom_EXPORT virtual
    ~geom_GordonSurface();

public:

  mobiusGeom_EXPORT virtual void
    Dump(std::stringstream& stream) const;

public:

  mobiusGeom_EXPORT virtual void
    Bounds(double& xMin, double& xMax,
           double& yMin, double& yMax,
           double& zMin, double& zMax) const;

public:

  mobiusGeom_EXPORT virtual double
    MinParameter_U() const;

  mobiusGeom_EXPORT virtual double
    MaxParameter_U() const;

  mobiusGeom_EXPORT virtual double
    MinParameter_V() const;

  mobiusGeom_EXPORT virtual double
    MaxParameter_V() const;

  mobiusGeom_EXPORT virtual void
    Eval(const double u,
         const double v,
         core_XYZ&    C) const;

private:

  int get_n() const { return (int) (m_UCurves.size() - 1); }

  int get_m() const { return (int) (m_VCurves.size() - 1); }

private:

  //std::vector<double> u_knots() const;

  //std::vector<double> v_knots() const;

  double L_ks(const int                  k,
              const int                  s,
              const double               t,
              const std::vector<double>& t_knots) const;

  double DL_ks(const int                  k,
               const int                  s,
               const double               t,
               const std::vector<double>& t_knots) const;

  double A_ks(const int k,
              const int s,
              const double t,
              const std::vector<double>& t_knots) const;

  double B_ks(const int                  k,
              const int                  s,
              const double               t,
              const std::vector<double>& t_knots) const;

  double DA_ks(const int                  k,
               const int                  s,
               const double               t,
               const std::vector<double>& t_knots) const;

private:

  //! Iso U curves.
  std::vector< ptr<curve> > m_UCurves;

  //! Iso V curves.
  std::vector< ptr<curve> > m_VCurves;

  //! Grid of parameters.
  TGrid m_grid;

};

//! Convenience shortcut.
typedef geom_GordonSurface gordon_surf;

};

#endif
