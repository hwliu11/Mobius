//-----------------------------------------------------------------------------
// Created on: 20 September 2021
//-----------------------------------------------------------------------------
// Copyright (c) 2021-present, Sergey Slyadnev
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

#ifndef poly_SurfAdapter_h
#define poly_SurfAdapter_h

// Poly includes
#include <mobius/poly.h>

// Core includes
#include <mobius/core_OBJECT.h>
#include <mobius/core_UV.h>
#include <mobius/core_XYZ.h>

namespace mobius {

//! \ingroup MOBIUS_POLY
//!
//! Adapter class connecting the Poly entities with the analytical surfaces.
class poly_SurfAdapter : public core_OBJECT
{
public:

  //! Inverts the passed `xyz` point onto an analytical surface corresponding
  //! to the curved domain with the `tag` ID.
  //! \param[in]  tag the domain ID.
  //! \param[in]  xyz the point to invert.
  //! \param[out] uv  the UV coordinates of the projection point.
  //! \return false if inversion is not done or not successful.
  virtual bool
    InvertPoint(const int    tag,
                const t_xyz& xyz,
                t_uv&        uv) = 0;

};

}

#endif
