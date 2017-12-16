//-----------------------------------------------------------------------------
// Created on: 09 March 2015
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef geom_SkinSurface_HeaderFile
#define geom_SkinSurface_HeaderFile

// Geometry includes
#include <mobius/geom_BSplineCurve.h>
#include <mobius/geom_BSplineSurface.h>

// BSpl includes
#include <mobius/bspl_KnotsSelection.h>
#include <mobius/bspl_ParamsSelection.h>

// STL includes
#include <vector>

namespace mobius {

//! Creates B-surface passing through the given series of compatible (!)
//! B-curves.
class geom_SkinSurface
{
public:

  //! Error codes.
  enum ErrCode
  {
    ErrCode_NoError = 0,
    ErrCode_NotInitialized,
    ErrCode_NotDone,
    ErrCode_NotEnoughCurves,
    ErrCode_NullCurvePassed,
    ErrCode_NotCompatibleCurves_Degree,
    ErrCode_NotCompatibleCurves_Knots,
    ErrCode_BadVDegree,
    ErrCode_CannotSelectParameters,
    ErrCode_CannotSelectKnots,
    ErrCode_CannotInterpolateIsoU
  };

public:

  mobiusGeom_EXPORT
    geom_SkinSurface();

  mobiusGeom_EXPORT
    geom_SkinSurface(const std::vector< Ptr<bcurve> >& curves,
                     const int                         deg_V,
                     const bool                        unifyCurves);

public:

  mobiusGeom_EXPORT void
    Init(const std::vector< Ptr<bcurve> >& curves,
         const int                         deg_V,
         const bool                        unifyCurves);

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

// For maintenance:
public:

  std::vector< Ptr<bcurve> > IsoU_Curves;

private:

  std::vector< Ptr<bcurve> > m_curves;  //!< Curves to interpolate.
  int                        m_iDeg_V;  //!< V-degree of interpolant surface.
  bool                       m_bUnify;  //!< Indicates whether to unify curves.
  ErrCode                    m_errCode; //!< Error code.
  Ptr<bsurf>                 m_surface; //!< Interpolant surface.

};

};

#endif
