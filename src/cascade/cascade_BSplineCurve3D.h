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

//! Converter between OpenCascade and Mobius for B-curves.
class cascade_BSplineCurve3D
{
public:

  mobiusCascade_EXPORT
    cascade_BSplineCurve3D(const Ptr<bcurve>& mobiusCurve);

  mobiusCascade_EXPORT
    cascade_BSplineCurve3D(const Handle(Geom_BSplineCurve)& occtCurve);

  mobiusCascade_EXPORT
    ~cascade_BSplineCurve3D();

public:

  mobiusCascade_EXPORT void
    ReApproxMobius(const double        theTol3d,
                   const GeomAbs_Shape theOrder,
                   const int           theMaxSegments,
                   const int           theMaxDegree);

  mobiusCascade_EXPORT void
    DirectConvert();

public:

  mobiusCascade_EXPORT const Ptr<bcurve>&
    GetMobiusCurve() const;

  mobiusCascade_EXPORT const Handle(Geom_BSplineCurve)&
    GetOpenCascadeCurve() const;

  mobiusCascade_EXPORT bool
    IsDone() const;

  mobiusCascade_EXPORT double
    MaxError() const;

protected:

  mobiusCascade_EXPORT void
    convertToOpenCascade();

  mobiusCascade_EXPORT void
    convertToMobius();

private:

  //! Mobius curve.
  Ptr<bcurve> m_mobiusCurve;

  //! OCCT curve.
  Handle(Geom_BSplineCurve) m_occtCurve;

  //! Maximum achieved approximation error.
  double m_fMaxError;

  //! Indicates whether re-approximation succeeded or not.
  bool m_bIsDone;

};

};

#endif
