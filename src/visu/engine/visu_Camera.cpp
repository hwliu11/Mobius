//-----------------------------------------------------------------------------
// Created on: 18 December 2013
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
#include <mobius/visu_Camera.h>

// Standard includes
#include <math.h>

#define QrCAMERA_RotoStep  2.0*M_PI/180
#define QrCAMERA_ZoomCoeff 2.0
#define QrCAMERA_ClipCoeff 100.0

//! Constructor.
//! \param viewportWidth  [in] width of the viewport.
//! \param viewportHeight [in] height of the viewport.
//! \param pos_x          [in] X coordinate for Camera's position.
//! \param pos_y          [in] Y coordinate for Camera's position.
//! \param pos_z          [in] Z coordinate for Camera's position.
//! \param sceneSize      [in] characteristic size of scene.
//! \param rotoStyle      [in] style of rotation.
mobius::visu_Camera::visu_Camera(const int            viewportWidth,
                                 const int            viewportHeight,
                                 const double         pos_x,
                                 const double         pos_y,
                                 const double         pos_z,
                                 const double         sceneSize,
                                 const SceneRotoStyle rotoStyle)
: core_OBJECT()
{
  this->Init(viewportWidth, viewportHeight,
             pos_x, pos_y, pos_z,
             sceneSize, rotoStyle);
}

//! Destructor.
mobius::visu_Camera::~visu_Camera()
{
}

//! Initializes Camera.
//! \param viewportWidth  [in] width of the viewport.
//! \param viewportHeight [in] height of the viewport.
//! \param pos_x          [in] X coordinate for Camera's position.
//! \param pos_y          [in] Y coordinate for Camera's position.
//! \param pos_z          [in] Z coordinate for Camera's position.
//! \param sceneSize      [in] characteristic size of scene.
//! \param rotoStyle      [in] style of rotation.
void mobius::visu_Camera::Init(const int            viewportWidth,
                               const int            viewportHeight,
                               const double         pos_x,
                               const double         pos_y,
                               const double         pos_z,
                               const double         sceneSize,
                               const SceneRotoStyle rotoStyle)
{
  m_fPosX   = pos_x;
  m_fPosY   = pos_y;
  m_fPosZ   = pos_z;
  m_fFocalX = pos_x;
  m_fFocalY = pos_y;
  m_fFocalZ = 0.0;

  this->InitViewportSize(viewportWidth, viewportHeight);
  this->InitSceneSize(sceneSize);
  this->InitRotoStyle(rotoStyle);
}

//! Initializes Camera with viewport size.
//! \param viewportWidth [in] width of the viewport.
//! \param viewportHeight [in] height of the viewport.
void mobius::visu_Camera::InitViewportSize(const int viewportWidth,
                                           const int viewportHeight)
{
  m_iViewportW = viewportWidth;
  m_iViewportH = viewportHeight;
}

//! Initializes Camera with characteristic size of scene.
//! \param sceneSize [in] characteristic size of scene.
void mobius::visu_Camera::InitSceneSize(const double sceneSize)
{
  m_fSceneSize = sceneSize;
}

//! Initializes Camera with desired style of rotation.
//! \param rotoStyle [in] style of rotation to apply.
void mobius::visu_Camera::InitRotoStyle(const SceneRotoStyle rotoStyle)
{
  m_rotoStyle = rotoStyle;
}

//! Sets new position for Camera.
//! \param pos_x [in] X coordinate for Camera's position.
//! \param pos_y [in] Y coordinate for Camera's position.
//! \param pos_z [in] Z coordinate for Camera's position.
void mobius::visu_Camera::SetPosition(const double pos_x,
                                      const double pos_y,
                                      const double pos_z)
{
  // Affect transient properties
  m_fPosX = pos_x;
  m_fPosY = pos_y;
  m_fPosZ = pos_z;
}

//! Returns Camera's current rotation style.
//! \return current rotation style.
mobius::visu_Camera::SceneRotoStyle
  mobius::visu_Camera::RotoStyle() const
{
  return m_rotoStyle;
}

//! Sets new position for Camera.
//! \param pos [in] Camera's position.
void mobius::visu_Camera::SetPosition(const t_xyz& pos)
{
  this->SetPosition( pos.X(), pos.Y(), pos.Z() );
}

//! Returns current Camera's position.
//! \return position of the Camera.
mobius::t_xyz mobius::visu_Camera::Position() const
{
  return t_xyz(m_fPosX, m_fPosY, m_fPosZ);
}

//! Sets new focal point for Camera.
//! \param pnt_x [in] X coordinate for Camera's focal point.
//! \param pnt_y [in] Y coordinate for Camera's focal point.
//! \param pnt_z [in] Z coordinate for Camera's focal point.
void mobius::visu_Camera::SetFocalPoint(const double pnt_x,
                                        const double pnt_y,
                                        const double pnt_z)
{
  // Affect transient properties
  m_fFocalX = pnt_x;
  m_fFocalY = pnt_y;
  m_fFocalZ = pnt_z;
}

//! Sets new focal point for Camera.
//! \param focal_pnt [in] Camera's focal point.
void mobius::visu_Camera::SetFocalPoint(const t_xyz& focal_pnt)
{
  this->SetFocalPoint( focal_pnt.X(), focal_pnt.Y(), focal_pnt.Z() );
}

//! Returns current Camera's focal point.
//! \return focal point of the Camera.
mobius::t_xyz mobius::visu_Camera::FocalPoint() const
{
  return t_xyz(m_fFocalX, m_fFocalY, m_fFocalZ);
}

//! Sets Camera's rotation, presuming that Camera's position is being
//! rotated around the axis having origin in a focal point.
//! \param qn [in] rotation to set.
void mobius::visu_Camera::SetRotation(const core_Quaternion& qn)
{
  m_qn = qn;
}

//! Returns Camera's rotation.
//! \return quaternion representing the rotation.
const mobius::core_Quaternion& mobius::visu_Camera::Rotation() const
{
  return m_qn;
}

//! Moves Camera to the left.
void mobius::visu_Camera::MoveLeft()
{
  // Affect position
  t_xyz pos = this->Position();
  pos.SetX( pos.X() - this->translationStep() );
  this->SetPosition(pos);
}

//! Moves Camera to the right.
void mobius::visu_Camera::MoveRight()
{
  // Affect position
  t_xyz pos = this->Position();
  pos.SetX( pos.X() + this->translationStep() );
  this->SetPosition(pos);
}

//! Moves Camera up.
void mobius::visu_Camera::MoveUp()
{
  // Affect position
  t_xyz pos = this->Position();
  pos.SetY( pos.Y() + this->translationStep() );
  this->SetPosition(pos);
}

//! Moves Camera down.
void mobius::visu_Camera::MoveDown()
{
  // Affect position
  t_xyz pos = this->Position();
  pos.SetY( pos.Y() - this->translationStep() );
  this->SetPosition(pos);
}

//! Moves Camera nearer to its focus.
void mobius::visu_Camera::MoveNearer()
{
  // Affect position
  t_xyz pos = this->Position();
  pos.SetZ( pos.Z() - this->translationStep()*QrCAMERA_ZoomCoeff );
  this->SetPosition(pos);
}

//! Moves Camera farther from its focus.
void mobius::visu_Camera::MoveFarther()
{
  // Affect position
  t_xyz pos = this->Position();
  pos.SetZ( pos.Z() + this->translationStep()*QrCAMERA_ZoomCoeff );
  this->SetPosition(pos);
}

//! Rotates Camera around OX axis.
//! \param isReverse [in] indicates whether rotation should be reversed.
void mobius::visu_Camera::RotateX(const double isReverse)
{
  this->RotateAxis(t_xyz::OX(), isReverse);
}

//! Rotates Camera around OY axis.
//! \param isReverse [in] indicates whether rotation should be reversed.
void mobius::visu_Camera::RotateY(const double isReverse)
{
  this->RotateAxis(t_xyz::OY(), isReverse);
}

//! Rotates Camera around OY axis.
//! \param isReverse [in] indicates whether rotation should be reversed.
void mobius::visu_Camera::RotateZ(const double isReverse)
{
  this->RotateAxis(t_xyz::OZ(), isReverse);
}

//! Rotates Camera around the given axis.
//! \param axis      [in] rotation axis.
//! \param isReverse [in] indicates whether rotation should be reversed.
void mobius::visu_Camera::RotateAxis(const t_xyz&   axis,
                                     const double isReverse)
{
  // Prepare axis
  t_xyz Ax;
  if ( m_rotoStyle == SceneRotoStyle_AroundMobileAxes )
  {
    core_Quaternion axis_q = m_qn*axis*m_qn.Inverted();
    if ( !axis_q.AsVector(Ax) )
      throw std::exception("Cannot convert quaternion to 3D vector");
  }
  else // SceneRotoStyle_AroundFixedAxes
    Ax = axis;

  // Affect stored rotation
  core_Quaternion Q(Ax, isReverse ? -QrCAMERA_RotoStep : QrCAMERA_RotoStep);
  m_qn = Q*m_qn;
}

//! Applies Camera to the scene and pushes all matrices to stack.
void mobius::visu_Camera::Push() const
{
  /* ========================
   *  Set up clipping volume
   * ======================== */

  // Set up projection matrix
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  // Crop coefficient
  const double cc = this->cropCoeff();

  // Frame half-width
  const double frameHW = cc*m_fPosZ/2;

  // Establish clipping volume (left, right, bottom, top, near, far)
  const GLfloat aspectRatio = (GLfloat) m_iViewportW / (GLfloat) m_iViewportH;
  if ( m_iViewportW <= m_iViewportH )
    glOrtho(-frameHW + m_fPosX,
             frameHW + m_fPosX,
            -frameHW / aspectRatio + m_fPosY,
             frameHW / aspectRatio + m_fPosY,
             m_fPosZ*QrCAMERA_ClipCoeff, -m_fPosZ*QrCAMERA_ClipCoeff);
  else
    glOrtho(-frameHW * aspectRatio + m_fPosX,
             frameHW * aspectRatio + m_fPosX,
            -frameHW + m_fPosY,
             frameHW + m_fPosY,
             m_fPosZ*QrCAMERA_ClipCoeff, -m_fPosZ*QrCAMERA_ClipCoeff);

  /* =================
   *  Set up rotation
   * ================= */

  double mx[3][3];
  m_qn.AsMatrix3x3(mx);

  // Note that here we assemble a transposed matrix comparing to those
  // unpacked from quaternion
  GLfloat glmx[16] ={ (GLfloat) mx[0][0], (GLfloat) mx[1][0], (GLfloat) mx[2][0], 0.0f, // X Column
                      (GLfloat) mx[0][1], (GLfloat) mx[1][1], (GLfloat) mx[2][1], 0.0f, // Y Column
                      (GLfloat) mx[0][2], (GLfloat) mx[1][2], (GLfloat) mx[2][2], 0.0f, // Z Column
                                0.0f,               0.0f,               0.0f,     1.0f }; // Translation

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadMatrixf(glmx);
}

//! Pops all Camera's matrices from stack.
void mobius::visu_Camera::Pop() const
{
  glPopMatrix();
}

//! Returns crop coefficient for the Camera's frame depending on its
//! current position.
//! \return crop coefficient.
double mobius::visu_Camera::cropCoeff() const
{
  return 1.0;
}

//! Returns step for translation of view frustum.
//! \return translation step.
double mobius::visu_Camera::translationStep() const
{
  return m_fSceneSize / 120.0;
}
