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

#ifndef visu_Camera_HeaderFile
#define visu_Camera_HeaderFile

// visu includes
#include <mobius/visu.h>

// core includes
#include <mobius/core_Quaternion.h>
#include <mobius/core_Ptr.h>
#include <mobius/core_XYZ.h>

// STL includes
#include <vector>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Class representing Camera concept for Qr 3D viewer. Camera gives handy
//! interface for changing point of view on the modeling space. Manipulation
//! with camera is normally performed in the following way:
//!
//! 1. In default observation position (looking from direction of OZ, while
//!    OX points to east and OY points to north) Camera establishes clipping
//!    volume with frame size depending on its current distance from the focus
//!    point which is (0, 0, 0) by default.
//!
//! 2. Then the scene is moved regarding to the position of Camera.
//!
//! \todo complete description once all details are clarified
class visu_Camera : public core_OBJECT
{
public:

  //! Styles of scene rotation.
  enum SceneRotoStyle
  {
    //! Scene will rotate around fixed coordinate system, where OX passes
    //! from left to right, OY -- from bottom to top, and OZ points
    //! to ourselves.
    //!
    //! From Camera's point of view, we can think that Camera is being
    //! rotated around some moveable axes (inversion of point of view effect).
    SceneRotoStyle_AroundFixedAxes = 1,

    //! Scene will rotate around mobile coordinate system, so if you draw
    //! reference box of the scene, you will rotate around the axes which
    //! move along with the scene.
    //!
    //! From Camera's point of view, we can think that Camera is always being
    //! rotated around the same axes (inversion of point of view effect).
    SceneRotoStyle_AroundMobileAxes
  };

public:

  mobiusVisu_EXPORT
    visu_Camera(const int            viewportWidth = 0,
                  const int            viewportHeight = 0,
                  const double         pos_x = 0.0,
                  const double         pos_y = 0.0,
                  const double         pos_z = 0.0,
                  const double         sceneSize = 1.0,
                  const SceneRotoStyle rotoStyle = SceneRotoStyle_AroundMobileAxes);

  mobiusVisu_EXPORT virtual
    ~visu_Camera();

public:

  mobiusVisu_EXPORT virtual void
    Init(const int            viewportWidth,
         const int            viewportHeight,
         const double         pos_x,
         const double         pos_y,
         const double         pos_z,
         const double         sceneSize,
         const SceneRotoStyle rotoStyle);

  mobiusVisu_EXPORT virtual void
    InitViewportSize(const int viewportWidth,
                     const int viewportHeight);

  mobiusVisu_EXPORT virtual void
    InitSceneSize(const double sceneSize);

  mobiusVisu_EXPORT virtual void
    InitRotoStyle(const SceneRotoStyle rotoStyle);

public:

  mobiusVisu_EXPORT virtual SceneRotoStyle
    RotoStyle() const;

  mobiusVisu_EXPORT virtual void
    SetPosition(const double pos_x, const double pos_y, const double pos_z);

  mobiusVisu_EXPORT virtual void
    SetPosition(const t_xyz& pos);

  mobiusVisu_EXPORT virtual t_xyz
    Position() const;

  mobiusVisu_EXPORT virtual void
    SetFocalPoint(const double pnt_x, const double pnt_y, const double pnt_z);

  mobiusVisu_EXPORT virtual void
    SetFocalPoint(const t_xyz& focal_pnt);

  mobiusVisu_EXPORT virtual t_xyz
    FocalPoint() const;

  mobiusVisu_EXPORT virtual void
    SetRotation(const core_Quaternion& qn);

  mobiusVisu_EXPORT virtual const core_Quaternion&
    Rotation() const;

  mobiusVisu_EXPORT virtual void
    MoveLeft();

  mobiusVisu_EXPORT virtual void
    MoveRight();

  mobiusVisu_EXPORT virtual void
    MoveUp();

  mobiusVisu_EXPORT virtual void
    MoveDown();

  mobiusVisu_EXPORT virtual void
    MoveNearer();

  mobiusVisu_EXPORT virtual void
    MoveFarther();

  mobiusVisu_EXPORT virtual void
    RotateX(const double isReverse = false);

  mobiusVisu_EXPORT virtual void
    RotateY(const double isReverse = false);

  mobiusVisu_EXPORT virtual void
    RotateZ(const double isReverse = false);

  mobiusVisu_EXPORT virtual void
    RotateAxis(const t_xyz&   axis,
               const double isReverse);

public:

  virtual void
    Push() const;

  virtual void
    Pop() const;

protected:

  virtual double
    cropCoeff() const;

  virtual double
    translationStep() const;

private:

  double m_fPosX; //!< X coordinate of position.
  double m_fPosY; //!< Y coordinate of position.
  double m_fPosZ; //!< Z coordinate of position.

  double m_fFocalX; //!< X coordinate of focal point.
  double m_fFocalY; //!< Y coordinate of focal point.
  double m_fFocalZ; //!< Z coordinate of focal point.

  int  m_iViewportW;   //!< Width of the viewport.
  int  m_iViewportH;   //!< Height of the viewport.
  double m_fSceneSize; //!< Characteristic size of scene.

  core_Quaternion m_qn;        //!< Rotation in quaternion form.
  SceneRotoStyle  m_rotoStyle; //!< Rotation style.

};

}

#endif
