//-----------------------------------------------------------------------------
// Created on: 02 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

#ifndef core_XYZ_HeaderFile
#define core_XYZ_HeaderFile

// core includes
#include <mobius/core.h>

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
  double SetCoord(const int        idx,
                  const double val)
  {
    double* coord = NULL;

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
  //! \param tol3D [in] three-dimensional tolerance for comparison.
  //! \return true/false.
  bool IsOrigin(const double tol3D = 0) const
  {
    return ( fabs(m_fX) <= tol3D ) &&
           ( fabs(m_fY) <= tol3D ) &&
           ( fabs(m_fZ) <= tol3D );
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

  mobiusCore_EXPORT double
    Dot(const core_XYZ& XYZ) const;

  mobiusCore_EXPORT core_XYZ
    Cross(const core_XYZ& XYZ) const;

  mobiusCore_EXPORT double
    Angle(const core_XYZ& XYZ) const;

public:

  mobiusCore_EXPORT core_XYZ&
    operator=(const core_XYZ& XYZ);

  mobiusCore_EXPORT core_XYZ
    operator*(const double coeff) const;

  mobiusCore_EXPORT core_XYZ
    operator*=(const double coeff);

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

private:

  double m_fX; //!< X coordinate.
  double m_fY; //!< Y coordinate.
  double m_fZ; //!< Z coordinate.

};

//! \ingroup MOBIUS_CORE
//!
//! Convenience alias.
typedef core_XYZ xyz;

};

#endif
