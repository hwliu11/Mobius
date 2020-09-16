//-----------------------------------------------------------------------------
// Created on: 31 July 2020
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

#ifndef visu_ActorMesh_HeaderFile
#define visu_ActorMesh_HeaderFile

// visu includes
#include <mobius/visu_ActorInsensitiveSurface.h>

// poly includes
#include <mobius/poly_Mesh.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Actor for polygonal mesh.
class visu_ActorMesh : public visu_Actor
{
public:

  //! Ctor.
  //! \param[in] mesh mesh to draw.
  mobiusVisu_EXPORT
    visu_ActorMesh(const t_ptr<t_mesh>& mesh);

  //! Dtor.
  mobiusVisu_EXPORT virtual
    ~visu_ActorMesh();

public:

  //! Calculates the axis-aligned bounding box of this actor.
  //! \param[out] xMin min X.
  //! \param[out] xMax max X.
  //! \param[out] yMin min Y.
  //! \param[out] yMax max Y.
  //! \param[out] zMin min Z.
  //! \param[out] zMax max Z.
  mobiusVisu_EXPORT virtual void
    GetBounds(double& xMin, double& xMax,
              double& yMin, double& yMax,
              double& zMin, double& zMax) const;

  //! Draws mesh.
  mobiusVisu_EXPORT virtual void
    GL_Draw();

protected:

  //! Draws shaded mesh.
  mobiusVisu_EXPORT void
    drawShading() const;

protected:

  //! Mesh to draw.
  t_ptr<t_mesh> m_mesh;

  //! Coordinates of mesh nodes.
  std::vector<GLfloat> m_points;

  //! Indices of nodes representing triangles.
  std::vector<GLuint> m_indices;

};

}

#endif
