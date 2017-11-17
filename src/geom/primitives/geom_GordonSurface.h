//-----------------------------------------------------------------------------
// Created on: 10 March 2015
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef geom_GordonSurface_HeaderFile
#define geom_GordonSurface_HeaderFile

// Geometry includes
#include <mobius/geom_Curve.h>
#include <mobius/geom_Surface.h>

namespace mobius {

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
    geom_GordonSurface(const std::vector< Ptr<curve> >& U_curves,
                       const std::vector< Ptr<curve> >& V_curves,
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
  std::vector< Ptr<curve> > m_UCurves;

  //! Iso V curves.
  std::vector< Ptr<curve> > m_VCurves;

  //! Grid of parameters.
  TGrid m_grid;

};

//! Convenience shortcut.
typedef geom_GordonSurface gordon_surf;

};

#endif
