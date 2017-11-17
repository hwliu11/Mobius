//-----------------------------------------------------------------------------
// Created on: 26 October 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef geom_InterpolateCurve_HeaderFile
#define geom_InterpolateCurve_HeaderFile

// Geometry includes
#include <mobius/geom_BSplineCurve.h>

// BSpl includes
#include <mobius/bspl_KnotsSelection.h>
#include <mobius/bspl_ParamsSelection.h>

// STL includes
#include <vector>

namespace mobius {

//! Interpolates B-curve over the given collection of points.
class geom_InterpolateCurve
{
public:

  //! Error codes.
  enum ErrCode
  {
    ErrCode_NoError = 0,
    ErrCode_NotInitialized,
    ErrCode_NotDone,
    ErrCode_CannotSelectParameters,
    ErrCode_CannotSelectKnots,
    ErrCode_InterpolationFailed
  };

public:

  mobiusGeom_EXPORT
    geom_InterpolateCurve();

  mobiusGeom_EXPORT
    geom_InterpolateCurve(const std::vector<xyz>&    points,
                          const int                  deg,
                          const bspl_ParamsSelection paramsType,
                          const bspl_KnotsSelection  knotsType);

public:

  mobiusGeom_EXPORT void
    Init(const std::vector<xyz>&    points,
         const int                  deg,
         const bspl_ParamsSelection paramsType,
         const bspl_KnotsSelection  knotsType);

  mobiusGeom_EXPORT void
    Init(const std::vector<xyz>&    points,
         const xyz&                 D0,
         const xyz&                 Dn,
         const int                  deg,
         const bspl_ParamsSelection paramsType,
         const bspl_KnotsSelection  knotsType);

  mobiusGeom_EXPORT void
    Init(const std::vector<xyz>&    points,
         const xyz&                 D0,
         const xyz&                 Dn,
         const xyz&                 D20,
         const xyz&                 D2n,
         const int                  deg,
         const bspl_ParamsSelection paramsType,
         const bspl_KnotsSelection  knotsType);

  mobiusGeom_EXPORT void
    Perform();

public:

  mobiusGeom_EXPORT static bool
    Interp(const std::vector<xyz>& points,
           const int               n,
           const int               p,
           const double*           params,
           const double*           U,
           const int               m,
           const bool              has_start_deriv,
           const bool              has_end_deriv,
           const bool              has_start_deriv2,
           const bool              has_end_deriv2,
           const xyz&              D0,
           const xyz&              Dn,
           const xyz&              D20,
           const xyz&              D2n,
           Ptr<bcurve>&          crv);

public:

  //! Accessor for error code.
  //! \return error code.
  int ErrorCode() const
  {
    return m_errCode;
  }

  //! Accessor for the resulting curve.
  //! \return interpolant curve.
  const Ptr<bcurve>& Result() const
  {
    return m_curve;
  }

protected:

  int  last_index_poles() const;
  int  last_index_knots() const;
  bool has_start_deriv()  const;
  bool has_end_deriv()    const;
  bool has_start_deriv2() const;
  bool has_end_deriv2()   const;

protected:

  static int dimension(const int  n,
                       const bool has_start_deriv,
                       const bool has_end_deriv,
                       const bool has_start_deriv2,
                       const bool has_end_deriv2);

private:

  int                  m_iDeg;       //!< Degree of interpolant curve.
  ErrCode              m_errCode;    //!< Error code.
  std::vector<xyz>     m_points;     //!< Points to interpolate.
  xyz                  m_D0;         //!< Derivative D1 at the first point.
  xyz                  m_Dn;         //!< Derivative D1 at the last point.
  xyz                  m_D20;        //!< Derivative D2 at the first point.
  xyz                  m_D2n;        //!< Derivative D2 at the last point.
  bspl_ParamsSelection m_paramsType; //!< Parameterization type.
  bspl_KnotsSelection  m_knotsType;  //!< Knots selection type.
  Ptr<bcurve>          m_curve;      //!< Interpolant curve.

};

};

#endif
