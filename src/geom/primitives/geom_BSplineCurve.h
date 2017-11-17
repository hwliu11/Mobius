//-----------------------------------------------------------------------------
// Created on: 23 May 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef geom_BSplineCurve_HeaderFile
#define geom_BSplineCurve_HeaderFile

// Geometry includes
#include <mobius/geom_Curve.h>

// Core includes
#include <mobius/core_Smoothness.h>

namespace mobius {

//! B-spline curve in 3D.
//! Degree, number of knots and number of control points are related
//! by formula (m = n + p + 1), where m is the index of last knot,
//! p is degree and n is the index of the last control point.
//!
//! \todo provide more comments.
class geom_BSplineCurve : public geom_Curve
{
// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_BSplineCurve(const std::vector<xyz>& Poles,
                      const double*           U,
                      const int               nU,
                      const int               p);

  mobiusGeom_EXPORT
    geom_BSplineCurve(const std::vector<xyz>&    Poles,
                      const std::vector<double>& U,
                      const int                  p);

  mobiusGeom_EXPORT virtual
    ~geom_BSplineCurve();

public:

  mobiusGeom_EXPORT virtual void
    Bounds(double& xMin, double& xMax,
           double& yMin, double& yMax,
           double& zMin, double& zMax) const;

public:

  mobiusGeom_EXPORT virtual double
    MinParameter() const;

  mobiusGeom_EXPORT virtual double
    MaxParameter() const;

  mobiusGeom_EXPORT virtual void
    Eval(const double u,
         xyz&         P) const;

  mobiusGeom_EXPORT virtual void
    Eval_Dk(const double u,
            const int    k,
            xyz&         dkC) const;

  mobiusGeom_EXPORT virtual void
    Eval_Dk(double**     dN,
            const double u,
            const int    k,
            xyz&         dkC) const;

  mobiusGeom_EXPORT virtual double
    K(const double u) const;

  mobiusGeom_EXPORT virtual core_Smoothness
    Continuity() const;

public:

  mobiusGeom_EXPORT Ptr<geom_BSplineCurve>
    Copy() const;

  mobiusGeom_EXPORT bool
    InvertPoint(const xyz&   P,
                double&      param,
                const double prec = 1.0e-6) const;

  mobiusGeom_EXPORT bool
    InsertKnot(const double u,
               const int    num_times = 1);

  mobiusGeom_EXPORT bool
    InsertKnot(const double u,
               const int    num_times,
               int&         dest_span_idx);

  mobiusGeom_EXPORT bool
    RefineKnots(const std::vector<double>& X);

  mobiusGeom_EXPORT bool
    Split(const double                           u,
          std::vector< Ptr<geom_BSplineCurve> >& slices) const;

  mobiusGeom_EXPORT void
    ReparameterizeLinear(const double s_min,
                         const double s_max);

public:

  //! Accessor for the collection of poles.
  //! \return poles of B-spline curve.
  const std::vector<xyz>& Poles() const
  {
    return m_poles;
  }

  //! Accessor for the collection of poles.
  //! \return poles of B-spline curve.
  std::vector<xyz>& ChangePoles()
  {
    return m_poles;
  }

  //! Accessor for the knot vector.
  //! \return knot vector.
  const std::vector<double>& Knots() const
  {
    return m_U;
  }

  //! Returns the number of knots.
  //! \return number of knots.
  int NumOfKnots() const
  {
    return (int) m_U.size();
  }

  //! Returns the degree of the curve.
  //! \return degree.
  int Degree() const
  {
    return m_iDeg;
  }

private:

  void init(const std::vector<xyz>&    Poles,
            const std::vector<double>& U,
            const int                  p);

private:

  //! Poles of B-spline curve.
  std::vector<xyz> m_poles;

  //! Knot vector.
  std::vector<double> m_U;

  //! Degree.
  int m_iDeg;

};

//! Convenience shortcut.
typedef geom_BSplineCurve bcurve;

};

#endif
