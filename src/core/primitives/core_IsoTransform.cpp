//-----------------------------------------------------------------------------
// Created on: 05 September 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

// Own include
#include <mobius/core_IsoTransform.h>

//! Constructs unitary isometric transformation.
mobius::core_IsoTransform::core_IsoTransform()
{}

//! Constructs isometric transformation from pure rotation and translation
//! parts.
//! \param rotation    [in] quaternion representing rotation.
//! \param translation [in] spatial vector representing translation.
mobius::core_IsoTransform::core_IsoTransform(const core_Quaternion& rotation,
                                             const core_XYZ&        translation)
{
  m_Q = rotation;
  m_T = translation;
}

//! Applies transformation to the given point.
//! \param coord [in] point to transform.
//! \return transformed point.
mobius::core_XYZ mobius::core_IsoTransform::Apply(const core_XYZ& coord) const
{
  // Rotate
  core_XYZ rotated_vec;
  core_Quaternion rotated_qn = m_Q*coord*m_Q.Conjugated();

  if ( !rotated_qn.AsVector(rotated_vec) )
    throw std::exception("Quaternion does not specify rotation");

  // Translate
  core_XYZ transformed_vec = rotated_vec + m_T;

  // Return result
  return transformed_vec;
}
