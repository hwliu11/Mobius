//-----------------------------------------------------------------------------
// Created on: 05 August 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef cascade_BSplineCurve3D_HeaderFile
#define cascade_BSplineCurve3D_HeaderFile

// Cascade includes
#include <mobius/cascade.h>

// Geom includes
#include <mobius/geom_BSplineCurve.h>

// OCCT includes
#include <Geom_BSplineCurve.hxx>
#include <GeomAbs_Shape.hxx>

namespace mobius {

//! Approximation tool for 3D B-spline curves.
class cascade_BSplineCurve3D
{
public:

  mobiusCascade_EXPORT
    cascade_BSplineCurve3D(const Ptr<bcurve>& theCurve);

  mobiusCascade_EXPORT
    ~cascade_BSplineCurve3D();

public:

  mobiusCascade_EXPORT void
    ReApproxConvert(const double        theTol3d,
                    const GeomAbs_Shape theOrder,
                    const int           theMaxSegments,
                    const int           theMaxDegree);

  mobiusCascade_EXPORT void
    DirectConvert();

public:

  mobiusCascade_EXPORT const Ptr<bcurve>&
    Source() const;

  mobiusCascade_EXPORT bool
    IsDone() const;

  mobiusCascade_EXPORT const Handle(Geom_BSplineCurve)&
    Result() const;

  mobiusCascade_EXPORT double
    MaxError() const;

private:

  //! Source Mobius curve.
  Ptr<bcurve> m_srcCurve;

  //! Resulting OCCT curve.
  Handle(Geom_BSplineCurve) m_resCurve;

  //! Maximum achieved approximation error.
  double m_fMaxError;

  //! Indicates whether re-approximation succeeded or not.
  bool m_bIsDone;

};

};

#endif
