//-----------------------------------------------------------------------------
// Created on: 23 May 2019
//-----------------------------------------------------------------------------
// Copyright (c) 2019-present, Sergey Slyadnev
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

#ifndef core_Axis_HeaderFile
#define core_Axis_HeaderFile

// core includes
#include <mobius/core_XYZ.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Axis which is a vector with position.
class core_Axis
{
// Construction & destruction:
public:

  //! Default ctor.
  core_Axis() {}

  //! Complete ctor.
  //! \param[in] pos origin of the axis.
  //! \param[in] dir direction of the axis.
  core_Axis(const xyz& pos,
            const xyz& dir) : m_pos(pos), m_dir(dir) {}

public:

  //! \return position of the axis.
  const xyz& GetPosition() const
  {
    return m_pos;
  }

  //! Sets the position of the axis.
  //! \param[in] pos position coordinates to set.
  void SetPosition(const xyz& pos)
  {
    m_pos = pos;
  }

  //! \return direction of the axis.
  const xyz& GetDirection() const
  {
    return m_dir;
  }

  //! Sets the direction of the axis.
  //! \param[in] dir direction vector to set.
  void SetDirection(const xyz& dir)
  {
    m_dir = dir;
  }

private:

  xyz m_pos; //!< Position.
  xyz m_dir; //!< Direction.

};

//! \ingroup MOBIUS_CORE
//!
//! Convenience alias.
typedef core_Axis axis;

};

#endif
