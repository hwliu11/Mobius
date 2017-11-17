//-----------------------------------------------------------------------------
// Created on: 22 May 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef geom_Line_HeaderFile
#define geom_Line_HeaderFile

// Geometry includes
#include <mobius/geom_Curve.h>

namespace mobius {

//! Parametric straight line.
class geom_Line : public geom_Curve
{
// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_Line(const core_XYZ& origin,
              const core_XYZ& dir);

  mobiusGeom_EXPORT virtual
    ~geom_Line();

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
         core_XYZ&    P) const;

public:

  //! Accessor for origin.
  //! \return origin of the line.
  const core_XYZ& Origin() const
  {
    return m_origin;
  }

  //! Accessor for direction.
  //! \return direction vector of the line.
  const core_XYZ& Dir() const
  {
    return m_dir;
  }

private:

  //! Origin of the line.
  core_XYZ m_origin;

  //! Direction of the line.
  core_XYZ m_dir;

};

};

#endif
