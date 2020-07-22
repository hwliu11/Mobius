//-----------------------------------------------------------------------------
// Created on: 21 May 2014
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

// Own include
#include <mobius/visu_Utils.h>

//! Converts the passed Win coordinates to OpenGL world referential.
//! \param mouseX [in] value on OX axis passing from left to right.
//! \param mouseY [in] value on OY axis passing from top to bottom.
//! \param X [out] resulting X value in World's referential.
//! \param Y [out] resulting Y value in World's referential.
//! \param Z [out] resulting Z value in World's referential.
void mobius::visu_Utils::DisplayToWorld(const int mouseX, const int mouseY,
                                        GLdouble& X, GLdouble& Y, GLdouble& Z)
{
  // Get transformations
  GLdouble modelMatrix[16], projMatrix[16];
  glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
  glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);

  // Get viewport properties
  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);

  GLfloat dWinx = (GLfloat) mouseX;
  GLfloat dWiny = (GLfloat) viewport[3] - (GLfloat) mouseY; // OpenGL's OY axis passes from bottom to top
  GLfloat dWinz;

  // Access Z (depth) value
  glReadPixels(GLint(dWinx), GLint(dWiny), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &dWinz);

  // Convert
  gluUnProject(dWinx, dWiny, dWinz, modelMatrix, projMatrix, viewport, &X, &Y, &Z);
}
