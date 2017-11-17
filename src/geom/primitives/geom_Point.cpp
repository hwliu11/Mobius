//-----------------------------------------------------------------------------
// Created on: 23 May 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

// Own include
#include <mobius/geom_Point.h>

//! Default constructor. Initializes point co-ordinates with origin values:
//! (0, 0, 0).
mobius::geom_Point::geom_Point() : geom_Geometry()
{}

//! Constructor with coordinates.
//! \param x [in] x coordinate.
//! \param y [in] y coordinate.
//! \param z [in] z coordinate.
mobius::geom_Point::geom_Point(const double x,
                               const double y,
                               const double z)
: geom_Geometry()
{
  m_XYZ.SetX(x);
  m_XYZ.SetY(y);
  m_XYZ.SetZ(z);
}

//! Assignment constructor.
//! \param PP [in] point to assign to this one.
mobius::geom_Point::geom_Point(const geom_Point& PP)
: geom_Geometry()
{
  this->operator=(PP);
}

//! Destructor.
mobius::geom_Point::~geom_Point()
{}

//! Returns bounds of the 3D point.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::geom_Point::Bounds(double& xMin, double& xMax,
                                double& yMin, double& yMax,
                                double& zMin, double& zMax) const
{
  // Bounding box is degenerated
  xMin = xMax = m_XYZ.X();
  yMin = yMax = m_XYZ.Y();
  zMin = zMax = m_XYZ.Z();
}

//! Multiplies copy of point by the passed scalar value.
//! \param coeff [in] scalar value to multiply point by.
//! \return resulting point.
mobius::geom_Point
  mobius::geom_Point::Multiplied(const double coeff) const
{
  return this->operator*(coeff);
}

//! Assignment operator.
//! \param PP [in] point to copy into this one.
//! \return this one.
mobius::geom_Point&
  mobius::geom_Point::operator=(const geom_Point& PP)
{
  m_XYZ = PP.m_XYZ;
  return *this;
}

//! Multiplies copy of point by the passed scalar value.
//! \param coeff [in] scalar value to multiply point by.
//! \return resulting point.
mobius::geom_Point
  mobius::geom_Point::operator*(const double coeff) const
{
  geom_Point P(*this);
  P.m_XYZ *= coeff;
  return P;
}

//! Adds the passed point to the copy of this one.
//! \param PP [in] point to add.
//! \return result of addition.
mobius::geom_Point
  mobius::geom_Point::operator+(const geom_Point& PP) const
{
  geom_Point P(*this);
  P.m_XYZ += PP.m_XYZ;
  return P;
}

//! Adds the passed point to this one.
//! \param PP [in] point to add.
//! \return result of addition.
mobius::geom_Point&
  mobius::geom_Point::operator+=(const geom_Point& PP)
{
  this->m_XYZ += PP.m_XYZ;
  return *this;
}

//! Adds the passed point to this one.
//! \param hPP [in] point to add.
//! \return result of addition.
mobius::geom_Point&
  mobius::geom_Point::operator+=(const Ptr<geom_Point>& hPP)
{
  return this->operator+=( *hPP.Access() );
}

//! Subtracts the passed point from the copy of this one.
//! \param PP [in] point to subtract.
//! \return result of subtraction.
mobius::geom_Point
  mobius::geom_Point::operator-(const geom_Point& PP) const
{
  geom_Point P(*this);
  P.m_XYZ -= PP.m_XYZ;
  return P;
}

//! Subtracts the passed point from this one.
//! \param PP [in] point to subtract.
//! \return result of subtraction.
mobius::geom_Point&
  mobius::geom_Point::operator-=(const geom_Point& PP)
{
  this->m_XYZ -= PP.m_XYZ;
  return *this;
}

//! Subtracts the passed point from this one.
//! \param hPP [in] point to subtract.
//! \return result of subtraction.
mobius::geom_Point&
  mobius::geom_Point::operator-=(const Ptr<geom_Point>& hPP)
{
  return this->operator-=( *hPP.Access() );
}
