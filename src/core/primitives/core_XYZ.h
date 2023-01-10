//-----------------------------------------------------------------------------
// Created on: 02 November 2013
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

#ifndef core_XYZ_HeaderFile
#define core_XYZ_HeaderFile

// core includes
#include <mobius/core_Precision.h>

// STL includes
#include <float.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Geometric primitive for 3D point.
class core_XYZ
{
// Static API:
public:

  //! \return number of coordinates.
  static int num_coordinates()
  {
    return 3;
  }

  mobiusCore_EXPORT static core_XYZ
    O();

  mobiusCore_EXPORT static core_XYZ
    OX();

  mobiusCore_EXPORT static core_XYZ
    OY();

  mobiusCore_EXPORT static core_XYZ
    OZ();

  mobiusCore_EXPORT static bool
    AreSamePlane(const std::vector<core_XYZ>& dirs,
                 const double                 prec = core_Precision::Resolution3D());

  mobiusCore_EXPORT static bool
    AreSpanningWholeSpace(const std::vector<core_XYZ>& dirs,
                          const double                 prec = core_Precision::Resolution3D());

// Construction & destruction:
public:

  //! Default constructor. Initializes point coordinates with origin values:
  //! (0, 0, 0).
  core_XYZ()
  {
    m_fX = m_fY = m_fZ = 0.0;
  }

  //! Constructor with coordinates.
  //! \param x [in] first coordinate.
  //! \param y [in] second coordinate.
  //! \param z [in] third coordinate.
  core_XYZ(const double x,
           const double y,
           const double z)
  {
    m_fX = x;
    m_fY = y;
    m_fZ = z;
  }

  //! Assignment constructor.
  //! \param XYZ [in] point to assign to this one.
  core_XYZ(const core_XYZ& XYZ)
  {
    m_fX = XYZ.X();
    m_fY = XYZ.Y();
    m_fZ = XYZ.Z();
  }

  //! Destructor.
  ~core_XYZ() {}

public:

  //! Returns X coordinate of the 3D point.
  //! \return X coordinate.
  double X() const
  {
    return m_fX;
  }

  //! Sets X coordinate.
  //! \param x [in] value to set.
  void SetX(const double x)
  {
    m_fX = x;
  }

  //! Returns Y coordinate of the 3D point.
  //! \return Y coordinate.
  double Y() const
  {
    return m_fY;
  }

  //! Sets Y coordinate.
  //! \param y [in] value to set.
  void SetY(const double y)
  {
    m_fY = y;
  }

  //! Returns Z coordinate of the 3D point.
  //! \return Z coordinate.
  double Z() const
  {
    return m_fZ;
  }

  //! Sets Z coordinate.
  //! \param z [in] value to set.
  void SetZ(const double z)
  {
    m_fZ = z;
  }

  //! Sets all coordinates of this class equal to the coordinates of the
  //! passed triple.
  //! \param[in] other other triple.
  void SetXYZ(const core_XYZ& other)
  {
    m_fX = other.m_fX;
    m_fY = other.m_fY;
    m_fZ = other.m_fZ;
  }

  //! Returns coordinate by its 0-based index.
  //! \param idx [in] 0 for X, 1 for Y, 2 for Z.
  //! \return requested coordinate.
  double Coord(const int idx) const
  {
    if ( idx == 0 )
      return this->X();

    if ( idx == 1 )
      return this->Y();

    if ( idx == 2 )
      return this->Z();

    return FLT_MAX;
  }

  //! Updates coordinate having the specified 0-based index with the
  //! passed value.
  //! \param idx [in] 0 for X, 1 for Y, 2 for Z.
  //! \param val [in] value to set.
  double SetCoord(const int    idx,
                  const double val)
  {
    double* coord = nullptr;

    if ( idx == 0 )
      coord = &m_fX;
    if ( idx == 1 )
      coord = &m_fY;
    if ( idx == 2 )
      coord = &m_fZ;

    if ( coord )
      *coord = val;

    return FLT_MAX;
  }

  //! Returns true if this vector is zero.
  //! \param[in] tol3D three-dimensional tolerance for comparison.
  //! \return true/false.
  bool IsOrigin(const double tol3D = 0.) const
  {
    return ( fabs(m_fX) <= tol3D ) &&
           ( fabs(m_fY) <= tol3D ) &&
           ( fabs(m_fZ) <= tol3D );
  }

  //! Checks if this coordinate tuple is equal to the passed one.
  //! \param[in] XYZ   other point to compare this one with.
  //! \param[in] tol3D three-dimensional tolerance for comparison.
  //! \return true in case of equality, false -- otherwise.
  bool IsEqual(const core_XYZ& XYZ,
               const double    tol3D = 0.) const
  {
    return ( fabs( m_fX - XYZ.X() ) <= tol3D ) &&
           ( fabs( m_fY - XYZ.Y() ) <= tol3D ) &&
           ( fabs( m_fZ - XYZ.Z() ) <= tol3D );
  }

  //! Compute component-wise minimum of two vectors.
  //! \param[in] XYZ other tuple to compare this one with.
  //! \return new tuple with min components.
  core_XYZ CWiseMin(const core_XYZ& XYZ) const
  {
    return core_XYZ(m_fX < XYZ.m_fX ? m_fX : XYZ.m_fX,
                    m_fY < XYZ.m_fY ? m_fY : XYZ.m_fY,
                    m_fZ < XYZ.m_fZ ? m_fZ : XYZ.m_fZ);
  }

  //! Compute component-wise maximum of two vectors.
  //! \param[in] XYZ other tuple to compare this one with.
  //! \return new tuple with max components.
  core_XYZ CWiseMax(const core_XYZ& XYZ) const
  {
    return core_XYZ(m_fX > XYZ.m_fX ? m_fX : XYZ.m_fX,
                    m_fY > XYZ.m_fY ? m_fY : XYZ.m_fY,
                    m_fZ > XYZ.m_fZ ? m_fZ : XYZ.m_fZ);
  }

  //! \return max components of the tuple.
  double GetMaxComponent() const
  {
    return m_fX > m_fY ? (m_fX > m_fZ ? m_fX : m_fZ)
                       : (m_fY > m_fZ ? m_fY : m_fZ);
  }

public:

  mobiusCore_EXPORT double
    Modulus() const;

  mobiusCore_EXPORT double
    SquaredModulus() const;

  mobiusCore_EXPORT core_XYZ
    Multiplied(const double coeff) const;

  mobiusCore_EXPORT void
    Normalize();

  mobiusCore_EXPORT core_XYZ
    Normalized() const;

  mobiusCore_EXPORT void
    Reverse();

  mobiusCore_EXPORT core_XYZ
    Reversed() const;

  mobiusCore_EXPORT double
    Dot(const core_XYZ& XYZ) const;

  mobiusCore_EXPORT core_XYZ
    Cross(const core_XYZ& XYZ) const;

  mobiusCore_EXPORT double
    Angle(const core_XYZ& XYZ) const;

  mobiusCore_EXPORT double
    AngleWithRef(const core_XYZ& other,
                 const core_XYZ& ref) const;

public:

  mobiusCore_EXPORT bool
    operator==(const core_XYZ& XYZ) const;

  mobiusCore_EXPORT core_XYZ&
    operator=(const core_XYZ& XYZ);

  mobiusCore_EXPORT core_XYZ
    operator*(const double coeff) const;

  mobiusCore_EXPORT core_XYZ
    operator*=(const double coeff);

  mobiusCore_EXPORT double
    operator*(const core_XYZ& XYZ) const;

  mobiusCore_EXPORT core_XYZ
    operator^(const core_XYZ& XYZ) const;

  mobiusCore_EXPORT core_XYZ
    operator/(const double coeff) const;

  mobiusCore_EXPORT core_XYZ
    operator/=(const double coeff);

  mobiusCore_EXPORT core_XYZ
    operator+(const core_XYZ& XYZ) const;

  mobiusCore_EXPORT core_XYZ&
    operator+=(const core_XYZ& XYZ);

  mobiusCore_EXPORT core_XYZ
    Invert() const;

  mobiusCore_EXPORT core_XYZ
    operator-(const core_XYZ& XYZ) const;

  mobiusCore_EXPORT core_XYZ&
    operator-=(const core_XYZ& XYZ);

  mobiusCore_EXPORT void
    Transform(const double (&mx)[3][3]);

private:

  double m_fX; //!< X coordinate.
  double m_fY; //!< Y coordinate.
  double m_fZ; //!< Z coordinate.

};

//! \ingroup MOBIUS_CORE
//!
//! Multiplication operator which allows prefix form.
//! \param[in] coeff  coefficient to multiply by.
//! \param[in] coords coordinates tuple.
//! \return multiplication result.
inline core_XYZ operator*(const double coeff, const core_XYZ& coords)
{
  core_XYZ r = coords*coeff;
  return r;
}

//! \ingroup MOBIUS_CORE
//!
//! Convenience alias.
typedef core_XYZ t_xyz;

}

#endif
