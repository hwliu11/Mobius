//-----------------------------------------------------------------------------
// Created on: 18 September 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2013-present, Sergey Slyadnev
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

#ifndef poly_Vertex_HeaderFile
#define poly_Vertex_HeaderFile

// Poly includes
#include <mobius/poly_Handles.h>

// Core includes
#include <mobius/core_XYZ.h>

namespace mobius {

//! \ingroup MOBIUS_POLY
//!
//! Vertex entity.
class poly_Vertex
{
public:

  //! Default ctor.
  mobiusPoly_EXPORT
    poly_Vertex();

  //! Constructor.
  //! \param[in] coords coordinates to set.
  mobiusPoly_EXPORT
    poly_Vertex(const core_XYZ& coords);

  //! Constructor.
  //! \param[in] x coordinate x of the vertex.
  //! \param[in] y coordinate y of the vertex.
  //! \param[in] z coordinate z of the vertex.
  mobiusPoly_EXPORT
    poly_Vertex(const double x,
                const double y,
                const double z);

public:

  double X() const { return m_coords.X(); } //!< \return X coordinate.
  double Y() const { return m_coords.Y(); } //!< \return Y coordinate.
  double Z() const { return m_coords.Z(); } //!< \return Z coordinate.

  //! Type conversion operator.
  operator core_XYZ() const { return m_coords; }

  //! \return non-const reference to the vertex coordinates.
  core_XYZ& ChangeCoords()
  {
    return m_coords;
  }

protected:

  core_XYZ m_coords; //!< Coordinates of the vertex.

};

}

#endif
