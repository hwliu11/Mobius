//-----------------------------------------------------------------------------
// Created on: 12 December 2019
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

#ifndef poly_DistanceFunc_HeaderFile
#define poly_DistanceFunc_HeaderFile

// Poly includes
#include <mobius/poly_RealFunc.h>

namespace mobius {

//! \ingroup MOBIUS_POLY
//!
//! Abstract distance function. The derived classes should implement one or
//! another way of distance computation. It is recommended to use accelerating
//! structures such as BVH for the fast distance computation.
class poly_DistanceFunc : public poly_RealFunc
{
public:

  //! Type of distance to measure.
  enum Mode
  {
    Mode_Signed = 0, //!< Signed distance.
    Mode_Unsigned    //!< Unsigned distance.
  };

public:

  //! Ctor.
  //! \param[in] mode signed/unsigned.
  mobiusPoly_EXPORT
    poly_DistanceFunc(const Mode mode);

public:

  //! \return distance computation mode.
  Mode GetMode() const
  {
    return m_mode;
  }

protected:

  //! Type of distance.
  Mode m_mode;

};

}

#endif
