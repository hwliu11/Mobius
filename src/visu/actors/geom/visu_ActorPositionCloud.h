//-----------------------------------------------------------------------------
// Created on: 29 January 2014
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

#ifndef visu_ActorPositionCloud_HeaderFile
#define visu_ActorPositionCloud_HeaderFile

// visu includes
#include <mobius/visu_Actor.h>
#include <mobius/visu_ColorSelector.h>
#include <mobius/visu_DataPositionCloud.h>

// geom includes
#include <mobius/geom_PositionCloud.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Class representing OpenGL Actor dedicated to visualization of Position
//! Cloud.
class visu_ActorPositionCloud : public visu_Actor
{
public:

  mobiusVisu_EXPORT
    visu_ActorPositionCloud(const t_ptr<t_pcloud>&        Cloud,
                            const visu_ColorRGB<GLubyte>& Color);

  mobiusVisu_EXPORT virtual
    ~visu_ActorPositionCloud();

public:

  mobiusVisu_EXPORT virtual void
    GetBounds(double& xMin, double& xMax,
              double& yMin, double& yMax,
              double& zMin, double& zMax) const;

  mobiusVisu_EXPORT virtual void
    GL_Draw();

public:

  mobiusVisu_EXPORT virtual t_ptr<visu_DataSet>
    IntersectWithLine(const t_ptr<geom_Line>& ray);

  mobiusVisu_EXPORT virtual void
    SetHiliData(const t_ptr<visu_DataSet>&);

  mobiusVisu_EXPORT virtual t_ptr<visu_DataSet>
    GetHiliData() const;

  mobiusVisu_EXPORT virtual void
    ClearHiliData();

  mobiusVisu_EXPORT virtual void
    GL_Hili() const;

public:

  //! Sets point size.
  //! \param ps [in] point size to set.
  void SetPointSize(const GLfloat ps)
  {
    m_fPointSize = ps;
  }

  //! Returns point size.
  //! \return point size.
  GLfloat PointSize() const
  {
    return m_fPointSize;
  }

  //! Sets point color.
  //! \param color [in] point color to set.
  void SetPointColor(const visu_ColorRGB<GLubyte>& color)
  {
    m_color = color;
  }

  //! Returns point color.
  //! \return point color.
  const visu_ColorRGB<GLubyte>& PointColor() const
  {
    return m_color;
  }

private:

  //! Point cloud to draw.
  t_ptr<t_pcloud> m_cloud;

  //! Color.
  visu_ColorRGB<GLubyte> m_color;

  //! Point size.
  GLfloat m_fPointSize;

  //! Vertices being rendered.
  GLfloat* m_pVertices;

// Highlighting
private:

  //! Portion of position cloud to highlight.
  t_ptr<visu_DataPositionCloud> m_hiliCloud;

};

}

#endif
