//-----------------------------------------------------------------------------
// Created on: 05 September 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

#ifndef core_IsoTransform_HeaderFile
#define core_IsoTransform_HeaderFile

// core includes
#include <mobius/core_Quaternion.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Isometric spatial transformation. Contains translation and rotation
//! components.
class core_IsoTransform
{
// Construction & destruction:
public:

  mobiusCore_EXPORT
    core_IsoTransform();

  mobiusCore_EXPORT
    core_IsoTransform(const core_Quaternion& rotation,
                      const core_XYZ&        translation);

public:

  //! Returns quaternion representing rotation.
  //! \return rotation part of transformation represented with quaternion.
  const core_Quaternion& Rotation() const
  {
    return m_Q;
  }

  //! Sets rotation part of transformation as quaternion.
  //! \param rotation [in] quaternion to set.
  void SetRotation(const core_Quaternion& rotation)
  {
    m_Q = rotation;
  }

  //! Returns translation part of transformation.
  //! \return translation vector.
  const core_XYZ& Translation() const
  {
    return m_T;
  }

  //! Sets translation part of transformation as spatial vector.
  //! \param translation [in] quaternion to set.
  void SetTranslation(const core_XYZ& translation)
  {
    m_T = translation;
  }

public:

  mobiusCore_EXPORT core_XYZ
    Apply(const core_XYZ& coord) const;

public:

  core_Quaternion m_Q; //!< Pure rotation part.
  core_XYZ        m_T; //!< Pure translation part.

};

};

#endif
