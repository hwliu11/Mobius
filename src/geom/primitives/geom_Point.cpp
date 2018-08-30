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
mobius::geom_Point::geom_Point(const adouble x,
                               const adouble y,
                               const adouble z)
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
void mobius::geom_Point::Bounds(adouble& xMin, adouble& xMax,
                                adouble& yMin, adouble& yMax,
                                adouble& zMin, adouble& zMax) const
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
  mobius::geom_Point::Multiplied(const adouble coeff) const
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
  mobius::geom_Point::operator*(const adouble coeff) const
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
  mobius::geom_Point::operator+=(const ptr<geom_Point>& hPP)
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
  mobius::geom_Point::operator-=(const ptr<geom_Point>& hPP)
{
  return this->operator-=( *hPP.Access() );
}
