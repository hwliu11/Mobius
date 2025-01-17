//-----------------------------------------------------------------------------
// Created on: 07 February 2014
//-----------------------------------------------------------------------------
// Copyright (c) 2014-present, Sergey Slyadnev
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

#ifndef geom_BSplineSurface_HeaderFile
#define geom_BSplineSurface_HeaderFile

// Core includes
#include <mobius/core_UV.h>

// Geometry includes
#include <mobius/geom_BSplineCurve.h>
#include <mobius/geom_Surface.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! B-spline surface.
//!
//! \todo provide more comments.
class geom_BSplineSurface : public geom_Surface
{
// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_BSplineSurface(const std::vector< std::vector<t_xyz> >& Poles,
                        const double*                            U,
                        const double*                            V,
                        const int                                nU,
                        const int                                nV,
                        const int                                p,
                        const int                                q);

  mobiusGeom_EXPORT
    geom_BSplineSurface(const std::vector< std::vector<t_xyz> >& Poles,
                        const std::vector<double>&               U,
                        const std::vector<double>&               V,
                        const int                                p,
                        const int                                q);

  mobiusGeom_EXPORT virtual
    ~geom_BSplineSurface();

public:

  mobiusGeom_EXPORT static core_Ptr<geom_BSplineSurface>
    Instance(const std::string& json);

  //! Constructs a B-spline surface for the given Bezier control points.
  //! \param[in] umin  the first `u` value.
  //! \param[in] umax  the last `u` value.
  //! \param[in] umin  the first `v` value.
  //! \param[in] umax  the last `v` value.
  //! \param[in] poles the passed poles.
  //! \return the constructed B-spline/Bezier surface.
  mobiusGeom_EXPORT static core_Ptr<geom_BSplineSurface>
    MakeBezier(const double                             umin,
               const double                             umax,
               const double                             vmin,
               const double                             vmax,
               const std::vector< std::vector<t_xyz> >& poles);

  //! Compares two passed B-surfaces with the prescribed tolerances.
  //! \param[in] F     first surface to compare.
  //! \param[in] G     second surface to compare.
  //! \param[in] tol2d tolerance to use in the two-dimensional space. This
  //!                  value is used to compare knot vectors.
  //! \param[in] tol3d tolerance to use in the three-dimensional space. This
  //!                  value is used to compare poles.
  //! \return true if the surfaces are structurally and geometrically equal.
  mobiusGeom_EXPORT static bool
    Compare(const core_Ptr<geom_BSplineSurface>& F,
            const core_Ptr<geom_BSplineSurface>& G,
            const double                         tol2d,
            const double                         tol3d);

public:

  mobiusGeom_EXPORT virtual void
    Dump(std::ostream* out) const;

  mobiusGeom_EXPORT std::string
    DumpJSON() const;

public:

  mobiusGeom_EXPORT virtual void
    GetBounds(double& xMin, double& xMax,
              double& yMin, double& yMax,
              double& zMin, double& zMax) const override;

public:

  mobiusGeom_EXPORT virtual double
    GetMinParameter_U() const override;

  mobiusGeom_EXPORT virtual double
    GetMaxParameter_U() const override;

  mobiusGeom_EXPORT virtual double
    GetMinParameter_V() const override;

  mobiusGeom_EXPORT virtual double
    GetMaxParameter_V() const override;

  mobiusGeom_EXPORT virtual void
    Eval(const double u,
         const double v,
         t_xyz&       S) const override;

public:

  mobiusGeom_EXPORT virtual void
    Eval_D1(const double u,
            const double v,
            t_xyz&       S,
            t_xyz&       dU,
            t_xyz&       dV) const;

  mobiusGeom_EXPORT virtual void
    Eval_D2(const double     u,
            const double     v,
            t_xyz&           S,
            t_xyz&           dU,
            t_xyz&           dV,
            t_xyz&           d2U,
            t_xyz&           d2V,
            t_xyz&           d2UV,
            t_ptr<t_alloc2d> alloc            = nullptr,
            const int        memBlockResultU  = -1,
            const int        memBlockResultV  = -1,
            const int        memBlockInternal = -1) const;

  mobiusGeom_EXPORT virtual core_Continuity
    GetContinuity() const;

public:

  mobiusGeom_EXPORT t_ptr<geom_BSplineSurface>
    Copy() const;

  mobiusGeom_EXPORT bool
    InvertPoint(const t_xyz&     P,
                t_uv&            params,
                const double     prec             = 1.0e-6,
                t_ptr<t_alloc2d> alloc            = nullptr,
                const int        memBlockResultU  = -1,
                const int        memBlockResultV  = -1,
                const int        memBlockInternal = -1) const;

  mobiusGeom_EXPORT bool
    InsertKnot_U(const double u,
                 const int    num_times = 1);

  mobiusGeom_EXPORT bool
    InsertKnot_V(const double v,
                 const int    num_times = 1);

  mobiusGeom_EXPORT t_ptr<geom_BSplineCurve>
    Iso_U(const double u) const;

  mobiusGeom_EXPORT t_ptr<geom_BSplineCurve>
    Iso_V(const double v) const;

  //! Finds max knot span in U direction of the surface.
  //! \return 0-based knot span index.
  mobiusGeom_EXPORT int
    FindMaxSpan_U() const;

  //! Finds max knot span in V direction of the surface.
  //! \return 0-based knot span index.
  mobiusGeom_EXPORT int
    FindMaxSpan_V() const;

  /*mobiusGeom_EXPORT core_XYZ
    NormUnit(const double u,
             const double v) const;*/

  //! When a flat thin elastic plate is deformed to the shape of a free-form
  //! surface, its bending energy (under some simplifying assumptions) is
  //! proportional to the area integral of sum of squared second derivatives.
  //! This sum represents "thin-plate" energies of the coordinates of surface
  //! mapping.
  //!
  //! \return calculated bending energy.
  mobiusGeom_EXPORT double
    ComputeBendingEnergy() const;

  //! Flips the parameterization of the B-surface so that its U direction
  //! becomes V direction and vice versa. As a result, the natural orientation
  //! of the surface becomes inverted.
  mobiusGeom_EXPORT void
    ExchangeUV();

  //! Concatenates the passed B-spline surface with this one if both patches
  //! are already compatible, i.e., they have the same degree and monotonically
  //! increasing knots. Concatenation is to be done in either U or V direction
  //! depending on the passed Boolean flag.
  //! \param[in] other the surface to concatenate with this one.
  //! \param[in] isU   the Boolean flag indicating whether to concatenate
  //!                  geometries in U or V direction.
  //! \return false if concatenation is not done, i.e., surfaces are not compatible.
  mobiusGeom_EXPORT bool
    ConcatenateCompatible(const t_ptr<geom_BSplineSurface>& other,
                          const bool                        isU);

public:

  //! Accessor for the collection of poles. The collection of poles is
  //! represented by two-dimensional vector, where 1-st dimension is
  //! dedicated to U, and second -- for V. E.g. Poles[0][1] addresses
  //! point at intersection of 0-th U-isoline and 1-st V-isoline.
  //! \return poles of B-spline surface.
  const std::vector< std::vector<t_xyz> >& GetPoles() const
  {
    return m_poles;
  }

  //! Returns the coordinates of a pole with the given indices.
  //! \param[in] i zero-based row index.
  //! \param[in] j zero-based column index.
  //! \return pole coordinates.
  const t_xyz& GetPole(const int i,
                       const int j) const
  {
    return m_poles[i][j];
  }

  //! Returns non-const reference to a pole with the given indices. Use this
  //! method to modify poles directly.
  //! \param[in] i zero-based row index.
  //! \param[in] j zero-based column index.
  //! \return reference to pole.
  t_xyz& ChangePole(const int i,
                    const int j)
  {
    return m_poles[i][j];
  }

  //! Sets new coordinates for a pole.
  //! \param[in] i   zero-based row index.
  //! \param[in] j   zero-based column index.
  //! \param[in] xyz pole coordinates to set.
  void SetPole(const int    i,
               const int    j,
               const t_xyz& xyz)
  {
    m_poles[i][j] = xyz;
  }

  //! Returns a knot value for the given zero-based index.
  //! \param[in] i zero-based index of a U knot to access.
  //! \return knot value.
  double GetKnot_U(const int i) const
  {
    return m_U[i];
  }

  //! Returns a knot value for the given zero-based index.
  //! \param[in] j zero-based index of a V knot to access.
  //! \return knot value.
  double GetKnot_V(const int j) const
  {
    return m_V[j];
  }

  //! Accessor for the knot vector in U dimension.
  //! \return knot vector.
  const std::vector<double>& GetKnots_U() const
  {
    return m_U;
  }

  //! Accessor for the knot vector in V dimension.
  //! \return knot vector.
  const std::vector<double>& GetKnots_V() const
  {
    return m_V;
  }

  //! Returns the number of knots in U dimension.
  //! \return number of knots.
  int GetNumOfKnots_U() const
  {
    return (int) m_U.size();
  }

  //! Returns the number of knots in V dimension.
  //! \return number of knots.
  int GetNumOfKnots_V() const
  {
    return (int) m_V.size();
  }

  //! \return number of interior knots in U direction.
  int GetNumOfInteriorKnots_U() const
  {
    int res = 0;
    for ( size_t k = m_iDegU + 1; k < m_U.size() - m_iDegU - 1; ++k )
      res++;

    return res;
  }

  //! \return number of interior knots in V direction.
  int GetNumOfInteriorKnots_V() const
  {
    int res = 0;
    for ( size_t k = m_iDegV + 1; k < m_V.size() - m_iDegV - 1; ++k )
      res++;

    return res;
  }

  //! Returns the number of poles in U direction.
  //! \return number of poles.
  int GetNumOfPoles_U() const
  {
    return (int) m_poles.size();
  }

  //! Returns the number of poles in V direction.
  //! \return number of poles.
  int GetNumOfPoles_V() const
  {
    return (int) m_poles[0].size();
  }

  //! Returns the degree of the surface in U dimension.
  //! \return degree.
  int GetDegree_U() const
  {
    return m_iDegU;
  }

  //! Returns the degree of the surface in V dimension.
  //! \return degree.
  int GetDegree_V() const
  {
    return m_iDegV;
  }

private:

  //! Initializes B-spline surface with complete data.
  //! \param[in] Poles control points.
  //! \param[in] U     knot vector in U dimension.
  //! \param[in] V     knot vector in V dimension.
  //! \param[in] p     degree of the B-spline basis functions in U dimension.
  //! \param[in] q     degree of the B-spline basis functions in V dimension.
  void init(const std::vector< std::vector<t_xyz> >& Poles,
            const std::vector<double>&               U,
            const std::vector<double>&               V,
            const int                                p,
            const int                                q);

  //! Finds the zero-based index of a knot span (in either U or V direction)
  //! which has the max length.
  //! \param[in] isU pass true if you are interested in U direction.
  //! \return zero-based span index.
  int findMaxSpan(const bool isU) const;

private:

  //! Poles of B-spline surface.
  std::vector< std::vector<t_xyz> > m_poles;

  //! Knot vector in U dimension.
  std::vector<double> m_U;

  //! Knot vector in V dimension.
  std::vector<double> m_V;

  //! Degree in U dimension.
  int m_iDegU;

  //! Degree in V dimension.
  int m_iDegV;

};

//! Convenience shortcut.
typedef geom_BSplineSurface t_bsurf;

}

#endif
