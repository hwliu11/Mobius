//-----------------------------------------------------------------------------
// Created on: 23 May 2013
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

#ifndef geom_BSplineCurve_HeaderFile
#define geom_BSplineCurve_HeaderFile

// Geometry includes
#include <mobius/geom_Curve.h>

// Core includes
#include <mobius/core_Continuity.h>
#include <mobius/core_HeapAlloc.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
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
                      const adouble*           U,
                      const int               nU,
                      const int               p);

  mobiusGeom_EXPORT
    geom_BSplineCurve(const std::vector<xyz>&    Poles,
                      const std::vector<adouble>& U,
                      const int                  p);

  mobiusGeom_EXPORT virtual
    ~geom_BSplineCurve();

public:

  mobiusGeom_EXPORT static core_Ptr<geom_BSplineCurve>
    Instance(const std::string& json);

public:

  mobiusGeom_EXPORT virtual void
    Bounds(adouble& xMin, adouble& xMax,
           adouble& yMin, adouble& yMax,
           adouble& zMin, adouble& zMax) const;

public:

  mobiusGeom_EXPORT virtual adouble
    MinParameter() const;

  mobiusGeom_EXPORT virtual adouble
    MaxParameter() const;

  mobiusGeom_EXPORT virtual void
    Eval(const adouble u,
         xyz&         P) const;

  mobiusGeom_EXPORT virtual void
    Eval_Dk(const adouble u,
            const int    k,
            xyz&         dkC,
            ptr<alloc2d> alloc            = NULL,
            const int    memBlockResult   = -1,
            const int    memBlockInternal = -1) const;

  mobiusGeom_EXPORT virtual void
    Eval_Dk(adouble**     dN,
            const adouble u,
            const int    k,
            xyz&         dkC,
            ptr<alloc2d> alloc            = NULL,
            const int    memBlockInternal = -1) const;

  mobiusGeom_EXPORT virtual adouble
    K(const adouble u) const;

  mobiusGeom_EXPORT virtual core_Continuity
    Continuity() const;

public:

  mobiusGeom_EXPORT ptr<geom_BSplineCurve>
    Copy() const;

  mobiusGeom_EXPORT bool
    InvertPoint(const xyz&   P,
                adouble&      param,
                const adouble prec = 1.0e-6) const;

  mobiusGeom_EXPORT bool
    InsertKnot(const adouble u,
               const int    num_times = 1);

  mobiusGeom_EXPORT bool
    InsertKnot(const adouble u,
               const int    num_times,
               int&         dest_span_idx);

  mobiusGeom_EXPORT bool
    RefineKnots(const std::vector<adouble>& X);

  mobiusGeom_EXPORT bool
    Split(const adouble                           u,
          std::vector< ptr<geom_BSplineCurve> >& slices) const;

  mobiusGeom_EXPORT void
    ReparameterizeLinear(const adouble s_min,
                         const adouble s_max);

  //! Calculates approximate strain energy of the B-curve. The strain
  //! energy is calculated as an integral of squared second derivative
  //! (instead of squared curvature).
  //!
  //! \return calculated strain energy.
  mobiusGeom_EXPORT adouble
    ComputeStrainEnergy() const;

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

  //! Returns the number of poles.
  //! \return number of poles.
  int NumOfPoles() const
  {
    return (int) m_poles.size();
  }

  //! Returns pole by its zero-based index.
  //! \param[in] poleIdx zero-based pole index.
  //! \return pole.
  const xyz& GetPole(const int poleIdx) const
  {
    return m_poles[poleIdx];
  }

  //! Sets new coordinates for a pole.
  //! \param[in] poleIdx zero-based pole index.
  //! \param[in] xyz     pole coordinates to set.
  //! \return pole.
  void SetPole(const int  poleIdx,
               const xyz& xyz)
  {
    m_poles[poleIdx] = xyz;
  }

  //! Accessor for the knot vector.
  //! \return knot vector.
  const std::vector<adouble>& Knots() const
  {
    return m_U;
  }

  //! Returns the number of knots.
  //! \return number of knots.
  int NumOfKnots() const
  {
    return (int) m_U.size();
  }

  //! Returns knot by its zero-based index.
  //! \param[in] knotIdx zero-based knot index.
  //! \return knot.
  adouble GetKnot(const int knotIdx) const
  {
    return m_U[knotIdx];
  }

  //! Returns the degree of the curve.
  //! \return degree.
  int Degree() const
  {
    return m_iDeg;
  }

private:

  void init(const std::vector<xyz>&    Poles,
            const std::vector<adouble>& U,
            const int                  p);

private:

  //! Poles of B-spline curve.
  std::vector<xyz> m_poles;

  //! Knot vector.
  std::vector<adouble> m_U;

  //! Degree.
  int m_iDeg;

};

//! Convenience shortcut.
typedef geom_BSplineCurve bcurve;

};

#endif
