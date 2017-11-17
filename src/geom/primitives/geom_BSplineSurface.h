//-----------------------------------------------------------------------------
// Created on: 07 February 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef geom_BSplineSurface_HeaderFile
#define geom_BSplineSurface_HeaderFile

// Geometry includes
#include <mobius/geom_BSplineCurve.h>
#include <mobius/geom_Surface.h>

namespace mobius {

//! B-spline surface.
//!
//! \todo provide more comments.
class geom_BSplineSurface : public geom_Surface
{
// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_BSplineSurface(const std::vector< std::vector<xyz> >& Poles,
                        const double*                          U,
                        const double*                          V,
                        const int                              nU,
                        const int                              nV,
                        const int                              p,
                        const int                              q);

  mobiusGeom_EXPORT
    geom_BSplineSurface(const std::vector< std::vector<xyz> >& Poles,
                        const std::vector<double>&             U,
                        const std::vector<double>&             V,
                        const int                              p,
                        const int                              q);

  mobiusGeom_EXPORT virtual
    ~geom_BSplineSurface();

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
         xyz&         C) const;

public:

  mobiusGeom_EXPORT Ptr<geom_BSplineCurve>
    Iso_U(const double u) const;

  mobiusGeom_EXPORT Ptr<geom_BSplineCurve>
    Iso_V(const double v) const;

  /*mobiusGeom_EXPORT core_XYZ
    NormUnit(const double u,
             const double v) const;*/

public:

  //! Accessor for the collection of poles. The collection of poles is
  //! represented by two-dimensional vector, where 1-st dimension is
  //! dedicated to U, and second -- for V. E.g. Poles[0][1] addresses
  //! point at intersection of 0-th U-isoline and 1-st V-isoline.
  //! \return poles of B-spline surface.
  const std::vector< std::vector<xyz> >& Poles() const
  {
    return m_poles;
  }

  //! Accessor for the knot vector in U dimension.
  //! \return knot vector.
  const std::vector<double>& Knots_U() const
  {
    return m_U;
  }

  //! Accessor for the knot vector in V dimension.
  //! \return knot vector.
  const std::vector<double>& Knots_V() const
  {
    return m_V;
  }

  //! Returns the number of knots in U dimension.
  //! \return number of knots.
  int NumOfKnots_U() const
  {
    return (int) m_U.size();
  }

  //! Returns the number of knots in V dimension.
  //! \return number of knots.
  int NumOfKnots_V() const
  {
    return (int) m_V.size();
  }

  //! Returns the degree of the surface in U dimension.
  //! \return degree.
  int Degree_U() const
  {
    return m_iDegU;
  }

  //! Returns the degree of the surface in V dimension.
  //! \return degree.
  int Degree_V() const
  {
    return m_iDegV;
  }

private:

  void init(const std::vector< std::vector<xyz> >& Poles,
            const std::vector<double>&             U,
            const std::vector<double>&             V,
            const int                              p,
            const int                              q);

private:

  //! Poles of B-spline surface.
  std::vector< std::vector<xyz> > m_poles;

  //! Knot vector in U dimension.
  std::vector<double> m_U;

  //! Knot vector in V dimension.
  std::vector<double> m_V;

  //! Degree in U dimension.
  int m_iDegU;

  //! Degree in V dimension.
  int m_iDegV;

};

//! Convenience shortcuts.
typedef geom_BSplineSurface bsurf;

};

#endif
