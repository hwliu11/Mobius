//-----------------------------------------------------------------------------
// Created on: 23 December 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef geom_InterpolateSurface_HeaderFile
#define geom_InterpolateSurface_HeaderFile

// Geometry includes
#include <mobius/geom_BSplineCurve.h>
#include <mobius/geom_BSplineSurface.h>
#include <mobius/geom_VectorField.h>

// BSpl includes
#include <mobius/bspl_KnotsSelection.h>
#include <mobius/bspl_ParamsSelection.h>

// STL includes
#include <vector>

namespace mobius {

//! Interpolates B-surface over the given collection of points.
class geom_InterpolateSurface
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
    ErrCode_CannotInterpolateIsoV,
    ErrCode_CannotInterpolateIsoU,
    ErrCode_PoorInitialGrid,
    ErrCode_NonRectangularGrid,
    ErrCode_NotEnoughVDerivs_Start_D1,
    ErrCode_NotEnoughVDerivs_End_D1,
    ErrCode_NotEnoughVDerivs_Start_D2,
    ErrCode_NotEnoughVDerivs_End_D2
  };

public:

  mobiusGeom_EXPORT
    geom_InterpolateSurface();

  mobiusGeom_EXPORT
    geom_InterpolateSurface(const std::vector< std::vector<xyz> >& points,
                            const int                              deg_U,
                            const int                              deg_V,
                            const bspl_ParamsSelection             paramsType,
                            const bspl_KnotsSelection              knotsType);

  mobiusGeom_EXPORT
    geom_InterpolateSurface(const std::vector< std::vector<xyz> >& points,
                            const int                              deg_U,
                            const int                              deg_V,
                            const Ptr<geom_VectorField>&           derivs_isoV_start_D1,
                            const Ptr<geom_VectorField>&           derivs_isoV_end_D1,
                            const bspl_ParamsSelection             paramsType,
                            const bspl_KnotsSelection              knotsType);

  mobiusGeom_EXPORT
    geom_InterpolateSurface(const std::vector< std::vector<xyz> >& points,
                            const int                              deg_U,
                            const int                              deg_V,
                            const Ptr<geom_VectorField>&           derivs_isoV_start_D1,
                            const Ptr<geom_VectorField>&           derivs_isoV_end_D1,
                            const Ptr<geom_VectorField>&           derivs_isoV_start_D2,
                            const Ptr<geom_VectorField>&           derivs_isoV_end_D2,
                            const bspl_ParamsSelection             paramsType,
                            const bspl_KnotsSelection              knotsType);

public:

  mobiusGeom_EXPORT void
    Init(const std::vector< std::vector<xyz> >& points,
         const int                              deg_U,
         const int                              deg_V,
         const bspl_ParamsSelection             paramsType,
         const bspl_KnotsSelection              knotsType);

  mobiusGeom_EXPORT void
    Init(const std::vector< std::vector<xyz> >& points,
         const int                              deg_U,
         const int                              deg_V,
         const Ptr<geom_VectorField>&           derivs_isoV_start_D1,
         const Ptr<geom_VectorField>&           derivs_isoV_end_D1,
         const bspl_ParamsSelection             paramsType,
         const bspl_KnotsSelection              knotsType);

  mobiusGeom_EXPORT void
    Init(const std::vector< std::vector<xyz> >& points,
         const int                              deg_U,
         const int                              deg_V,
         const Ptr<geom_VectorField>&           derivs_isoV_start_D1,
         const Ptr<geom_VectorField>&           derivs_isoV_end_D1,
         const Ptr<geom_VectorField>&           derivs_isoV_start_D2,
         const Ptr<geom_VectorField>&           derivs_isoV_end_D2,
         const bspl_ParamsSelection             paramsType,
         const bspl_KnotsSelection              knotsType);

  mobiusGeom_EXPORT void
    Perform();

public:

  //! Accessor for error code.
  //! \return error code.
  int ErrorCode() const
  {
    return m_errCode;
  }

  //! Accessor for the resulting surface.
  //! \return interpolant surface.
  const Ptr<bsurf>& Result() const
  {
    return m_surface;
  }

private:

  int                             m_iDeg_U;     //!< U-degree of interpolant surface.
  int                             m_iDeg_V;     //!< V-degree of interpolant surface.
  ErrCode                         m_errCode;    //!< Error code.
  std::vector< std::vector<xyz> > m_points;     //!< Points to interpolate.
  bspl_ParamsSelection            m_paramsType; //!< Parameterization type.
  bspl_KnotsSelection             m_knotsType;  //!< Knots selection type.
  Ptr<bsurf>                      m_surface;    //!< Interpolant surface.

// Derivative constraints:
private:

  Ptr<geom_VectorField> m_derivs_isoV_start_D1; //!< Leading derivative D1 constraints on V iso.
  Ptr<geom_VectorField> m_derivs_isoV_end_D1;   //!< Trailing derivative D1 constraints on V iso.

  Ptr<geom_VectorField> m_derivs_isoV_start_D2; //!< Leading derivative D2 constraints on V iso.
  Ptr<geom_VectorField> m_derivs_isoV_end_D2;   //!< Trailing derivative D2 constraints on V iso.

// For maintenance:
public:

  std::vector< Ptr<bcurve> > IsoV_Curves;
  std::vector< Ptr<bcurve> > ReperU_Curves;

};

};

#endif
