//-----------------------------------------------------------------------------
// Created on: 20 June 2014
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

#ifndef visu_ActorSurface_HeaderFile
#define visu_ActorSurface_HeaderFile

// Windows
#include <windows.h>

// GL includes
#include <gl/gl.h>
#include <gl/glu.h>

// visu includes
#include <mobius/visu_Actor.h>

// geom includes
#include <mobius/geom_Surface.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Base class for OpenGL Actors representing parameteric surfaces.
class visu_ActorSurface : public visu_Actor
{
protected:

  mobiusVisu_EXPORT
    visu_ActorSurface(const t_ptr<geom_Surface>& Surf);

  mobiusVisu_EXPORT virtual
    ~visu_ActorSurface();

public:

  mobiusVisu_EXPORT virtual void
    GetBounds(double& xMin, double& xMax,
              double& yMin, double& yMax,
              double& zMin, double& zMax) const;

protected:

  mobiusVisu_EXPORT virtual void
    GL_drawAxes() const;

protected:

  //! Surface to draw.
  t_ptr<geom_Surface> m_surf;

  //! Array of points filling surface interior. These points are simply
  //! queried from the surface at construction time. This is the most
  //! robust way to get idea of surface form.
  GLfloat* m_pFilling;

  //! Number of points in filling.
  GLint m_iNumFillingPoints;

// Local axes
protected:

  t_xyz m_axes_origin; //!< Origin of local axes.
  t_xyz m_axes_OX;     //!< Pointing in OX direction.
  t_xyz m_axes_OY;     //!< Pointing in OY direction.
  t_xyz m_axes_OZ;     //!< Pointing in OZ direction.

};

}

#endif
