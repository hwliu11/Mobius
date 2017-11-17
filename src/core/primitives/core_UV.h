//-----------------------------------------------------------------------------
// Created on: 02 November 2012
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

#ifndef core_UV_HeaderFile
#define core_UV_HeaderFile

// core includes
#include <mobius/core.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Geometric primitive for 2D point.
class core_UV
{
public:

  //! \return number of coordinates.
  static int num_coordinates()
  {
    return 2;
  }

// Construction & destruction:
public:

  mobiusCore_EXPORT
    core_UV();

  mobiusCore_EXPORT
    core_UV(const double u, const double v);

  mobiusCore_EXPORT
    core_UV(const core_UV& UV);

  mobiusCore_EXPORT virtual
    ~core_UV();

public:

  //! Returns U coordinate of the 2D point.
  //! \return U coordinate.
  double U() const
  {
    return m_fU;
  }

  //! Sets U coordinate.
  //! \param u [in] value to set.
  void SetU(const double u)
  {
    m_fU = u;
  }

  //! Returns V coordinate of the 2D point.
  //! \return V coordinate.
  double V() const
  {
    return m_fV;
  }

  //! Sets V coordinate.
  //! \param v [in] value to set.
  void SetV(const double v)
  {
    m_fV = v;
  }

  //! Returns coordinate by its 0-based index.
  //! \param idx [in] 0 for U, 1 for V.
  //! \return requested coordinate.
  double Coord(const int idx) const
  {
    if ( idx == 0 )
      return this->U();

    if ( idx == 1 )
      return this->V();

    return 0;
  }

  //! Updates coordinate having the specified 0-based index with the
  //! passed value.
  //! \param idx [in] 0 for U, 1 for V.
  //! \param val [in] value to set.
  double SetCoord(const int        idx,
                      const double val)
  {
    double* coord = NULL;

    if ( idx == 0 )
      coord = &m_fU;
    if ( idx == 1 )
      coord = &m_fV;

    if ( coord )
      *coord = val;

    return 0;
  }

public:

  mobiusCore_EXPORT double
    Modulus() const;

  mobiusCore_EXPORT double
    SquaredModulus() const;

  mobiusCore_EXPORT double
    Dot(const core_UV& UV) const;

public:

  mobiusCore_EXPORT core_UV&
    operator=(const core_UV& UV);

  mobiusCore_EXPORT core_UV
    operator*(const double coeff) const;

  mobiusCore_EXPORT core_UV
    operator*=(const double coeff);

  mobiusCore_EXPORT core_UV
    operator/(const double coeff) const;

  mobiusCore_EXPORT core_UV
    operator/=(const double coeff);

  mobiusCore_EXPORT core_UV
    operator+(const core_UV& UV) const;

  mobiusCore_EXPORT core_UV&
    operator+=(const core_UV& UV);

  mobiusCore_EXPORT core_UV
    Invert() const;

  mobiusCore_EXPORT core_UV
    operator-(const core_UV& UV) const;

  mobiusCore_EXPORT core_UV&
    operator-=(const core_UV& UV);

private:

  double m_fU; //!< U coordinate.
  double m_fV; //!< V coordinate.

};

//! \ingroup MOBIUS_CORE
//!
//! Convenience alias.
typedef core_UV uv;

//! \ingroup MOBIUS_CORE
//!
//! Convenience alias.
typedef core_UV core_XY;

//! \ingroup MOBIUS_CORE
//!
//! Convenience alias.
typedef core_XY xy;

};

#endif
