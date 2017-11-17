//-----------------------------------------------------------------------------
// Created on: 23 May 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef geom_Point_HeaderFile
#define geom_Point_HeaderFile

// Geometry includes
#include <mobius/geom_Geometry.h>

// Core includes
#include <mobius/core_XYZ.h>

namespace mobius {

//! Geometric primitive for 3D point.
//!
//! \todo complete description
class geom_Point : public geom_Geometry
{
// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_Point();

  mobiusGeom_EXPORT
    geom_Point(const double x,
               const double y,
               const double z);

  mobiusGeom_EXPORT
    geom_Point(const geom_Point& PP);

  mobiusGeom_EXPORT virtual
    ~geom_Point();

public:

  mobiusGeom_EXPORT virtual void
    Bounds(double& xMin, double& xMax,
           double& yMin, double& yMax,
           double& zMin, double& zMax) const;

public:

  //! Returns X co-ordinate of the 3D point.
  //! \return X co-ordinate.
  double X() const
  {
    return m_XYZ.X();
  }

  //! Returns Y co-ordinate of the 3D point.
  //! \return Y co-ordinate.
  double Y() const
  {
    return m_XYZ.Y();
  }

  //! Returns Z co-ordinate of the 3D point.
  //! \return Z co-ordinate.
  double Z() const
  {
    return m_XYZ.Z();
  }

public:

  mobiusGeom_EXPORT geom_Point
    Multiplied(const double coeff) const;

public:

  mobiusGeom_EXPORT geom_Point&
    operator=(const geom_Point& PP);

  mobiusGeom_EXPORT geom_Point
    operator*(const double coeff) const;

  mobiusGeom_EXPORT geom_Point
    operator+(const geom_Point& PP) const;

  mobiusGeom_EXPORT geom_Point&
    operator+=(const geom_Point& PP);

  mobiusGeom_EXPORT geom_Point&
    operator+=(const Ptr<geom_Point>& hPP);

  mobiusGeom_EXPORT geom_Point
    operator-(const geom_Point& PP) const;

  mobiusGeom_EXPORT geom_Point&
    operator-=(const geom_Point& PP);

  mobiusGeom_EXPORT geom_Point&
    operator-=(const Ptr<geom_Point>& hPP);

private:

  xyz m_XYZ; //!< XYZ primitive.

};

};

#endif
