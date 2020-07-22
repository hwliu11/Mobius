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

// Own include
#include <mobius/visu_ActorBezierOnRailsSurface.h>

//! Constructor.
//! \param Surf [in] surface to draw.
mobius::visu_ActorBezierOnRailsSurface::visu_ActorBezierOnRailsSurface(const t_ptr<geom_BezierOnRailsSurface>& Surf)
: visu_ActorInsensitiveSurface( Surf.Access() )
{
}

//! Destructor.
mobius::visu_ActorBezierOnRailsSurface::~visu_ActorBezierOnRailsSurface()
{
}

//! Draws surface.
void mobius::visu_ActorBezierOnRailsSurface::GL_Draw()
{
  /* ==========================
   *  Render points on surface
   * ========================== */

  #pragma region Filling
  glEnable(GL_POINT_SMOOTH);
  glPointSize(1);

    glColor3f(1.0f, 0.8f, 0.1f);
    glEnableClientState(GL_VERTEX_ARRAY);
      glVertexPointer(3, GL_FLOAT, 0, m_pFilling);
      glDrawArrays(GL_POINTS, 0, m_iNumFillingPoints);
    glDisableClientState(GL_VERTEX_ARRAY);

  glEnd();
  glDisable(GL_POINT_SMOOTH);
  #pragma endregion
}
