//-----------------------------------------------------------------------------
// Created on: 11 August 2013
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

// Windows
#include <windows.h>

// GL includes
#include <gl/gl.h>
#include <gl/glu.h>

// Own include
#include <mobius/visu_ActorCartesianAxes.h>

// Standard includes
#include <math.h>

//! Constructor.
//! \param axisLength [in] length of each axis.
mobius::visu_ActorCartesianAxes::visu_ActorCartesianAxes(const double axisLength)
: visu_ActorInsensitive (),
  m_fAxisLength         (axisLength)
{
}

//! Destructor.
mobius::visu_ActorCartesianAxes::~visu_ActorCartesianAxes()
{
}

//! Calculates boundary box for the Cartesian Axes Actor.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::visu_ActorCartesianAxes::GetBounds(double& xMin, double& xMax,
                                                double& yMin, double& yMax,
                                                double& zMin, double& zMax) const
{
  xMin = -m_fAxisLength;
  xMax =  m_fAxisLength;
  yMin = -m_fAxisLength;
  yMax =  m_fAxisLength;
  zMin = -m_fAxisLength;
  zMax =  m_fAxisLength;
}

//! Draws cartesian axes.
void mobius::visu_ActorCartesianAxes::GL_Draw()
{
  const GLfloat length     = (GLfloat) m_fAxisLength;
  const GLfloat tip_length = (GLfloat) (length / 15.0);
  const GLfloat ang        = (GLfloat) M_PI/6;

  #pragma region GL lines
  glLineWidth(2);
  glEnable(GL_LINE_SMOOTH);
  glBegin(GL_LINES);

    // OX
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(length, 0.0, 0.0);

    // OY
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, length, 0.0);

    // OZ
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, length);

  glEnd();
  glDisable(GL_LINE_SMOOTH);
  glLineWidth(1);
  #pragma endregion

  #pragma region GL tips
  glBegin(GL_TRIANGLES);

    // OX tip
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(length, 0.0, 0.0);
    glVertex3f(length-tip_length*cos(ang), tip_length*sin(ang), 0.0);
    glVertex3f(length-tip_length*cos(ang), 0.0, tip_length*sin(ang));
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(length, 0.0, 0.0);
    glVertex3f(length-tip_length*cos(ang), tip_length*sin(ang), 0.0);
    glVertex3f(length-tip_length*cos(ang), 0.0, -tip_length*sin(ang));
    glVertex3f(length, 0.0, 0.0);
    glVertex3f(length-tip_length*cos(ang), -tip_length*sin(ang), 0.0);
    glVertex3f(length-tip_length*cos(ang), 0.0, tip_length*sin(ang));
    glVertex3f(length, 0.0, 0.0);
    glVertex3f(length-tip_length*cos(ang), -tip_length*sin(ang), 0.0);
    glVertex3f(length-tip_length*cos(ang), 0.0, -tip_length*sin(ang));

    // OY tip
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0, length, 0.0);
    glVertex3f(tip_length*sin(ang), length-tip_length*cos(ang), 0.0);
    glVertex3f(0.0, length-tip_length*cos(ang), tip_length*sin(ang));
    glVertex3f(0.0, length, 0.0);
    glVertex3f(-tip_length*sin(ang), length-tip_length*cos(ang), 0.0);
    glVertex3f(0.0, length-tip_length*cos(ang), tip_length*sin(ang));
    glVertex3f(0.0, length, 0.0);
    glVertex3f(tip_length*sin(ang), length-tip_length*cos(ang), 0.0);
    glVertex3f(0.0, length-tip_length*cos(ang), -tip_length*sin(ang));
    glVertex3f(0.0, length, 0.0);
    glVertex3f(-tip_length*sin(ang), length-tip_length*cos(ang), 0.0);
    glVertex3f(0.0, length-tip_length*cos(ang), -tip_length*sin(ang));

    // OZ tip
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0, 0.0, length);
    glVertex3f(tip_length*sin(ang), 0.0, length-tip_length*cos(ang));
    glVertex3f(0.0, tip_length*sin(ang), length-tip_length*cos(ang));
    glVertex3f(0.0, 0.0, length);
    glVertex3f(-tip_length*sin(ang), 0.0, length-tip_length*cos(ang));
    glVertex3f(0.0, tip_length*sin(ang), length-tip_length*cos(ang));
    glVertex3f(0.0, 0.0, length);
    glVertex3f(tip_length*sin(ang), 0.0, length-tip_length*cos(ang));
    glVertex3f(0.0, -tip_length*sin(ang), length-tip_length*cos(ang));
    glVertex3f(0.0, 0.0, length);
    glVertex3f(-tip_length*sin(ang), 0.0, length-tip_length*cos(ang));
    glVertex3f(0.0, -tip_length*sin(ang), length-tip_length*cos(ang));

  glEnd();
  #pragma endregion

  #pragma region GL points
  glEnable(GL_POINT_SMOOTH);
  glPointSize(5);
  glBegin(GL_POINTS);

    // Origin
    glColor3f(1.0f, 1.0f, 1.0f);
    glVertex3f(0.0, 0.0, 0.0);

  glEnd();
  glDisable(GL_LINE_SMOOTH);
  #pragma endregion
}

//! Sets axis length to the passed value.
//! \param axisLength [in] value to set.
void mobius::visu_ActorCartesianAxes::SetAxisLength(const double axisLength)
{
  m_fAxisLength = axisLength;
}

//! Returns axis length.
//! \return axis length.
double mobius::visu_ActorCartesianAxes::AxisLength() const
{
  return m_fAxisLength;
}
