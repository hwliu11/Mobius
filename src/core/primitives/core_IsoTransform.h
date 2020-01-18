//-----------------------------------------------------------------------------
// Created on: 05 September 2014
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

}

#endif
