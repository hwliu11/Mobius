//-----------------------------------------------------------------------------
// Created on: 17 June 2013
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

#ifndef visu_ViewWindow_HeaderFile
#define visu_ViewWindow_HeaderFile

// visu includes
#include <mobius/visu_Camera.h>
#include <mobius/visu_CommandQueue.h>
#include <mobius/visu_CommandRepo.h>
#include <mobius/visu_Picker.h>
#include <mobius/visu_Scene.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Class representing OpenGL rendering window.
class visu_ViewWindow
{
private:

  //! Current interaction state.
  struct IState
  {
    int        MousePickedX;
    int        MousePickedY;
    int        MouseHoverX;
    int        MouseHoverY;
    int        MouseDraggedX;
    int        MouseDraggedY;
    t_xyz        WorldXYZ;
    bool       IsLButtonClicked;
    bool       IsRButtonClicked;
    bool       IsMButtonClicked;
    bool       IsCtrlPressed;
    bool       IsPositiveRotoX;
    bool       IsPositiveRotoY;

    IState() : MousePickedX(0),
               MousePickedY(0),
               MouseHoverX(0),
               MouseHoverY(0),
               MouseDraggedX(0),
               MouseDraggedY(0),
               IsLButtonClicked(false),
               IsMButtonClicked(false),
               IsRButtonClicked(false),
               IsCtrlPressed(false),
               IsPositiveRotoX(true),
               IsPositiveRotoY(true) {}
  };

public:

  mobiusVisu_EXPORT
    visu_ViewWindow(const t_ptr<visu_CommandQueue>& Queue,
                      const t_ptr<visu_CommandRepo>& CmdRepo);

  mobiusVisu_EXPORT virtual
    ~visu_ViewWindow();

public:

  mobiusVisu_EXPORT virtual void
    Close();

  mobiusVisu_EXPORT virtual bool
    Create(const int theLeft,
           const int theTop,
           const int theWidth,
           const int theHeight);

  mobiusVisu_EXPORT virtual void
    ResetScene();

  mobiusVisu_EXPORT virtual void
    Resize(const int theWidth,
           const int theHeight);

  mobiusVisu_EXPORT virtual void
    SwitchRotationStyle();

  mobiusVisu_EXPORT virtual void
    StartMessageLoop();

public:

  //! Accessor for the Scene object.
  //! \return requested Scene instance.
  inline const t_ptr<visu_Scene>& Scene() const
  {
    return m_scene;
  }

  //! Accessor for the Picker object.
  //! \return requested Picker instance.
  inline const t_ptr<visu_Picker>& Picker() const
  {
    return m_picker;
  }

private:

  static LRESULT WINAPI wndProcProxy(HWND hwnd,
                                     UINT message,
                                     WPARAM wparam,
                                     LPARAM lparam);

  LRESULT CALLBACK wndProc(HWND hwnd,
                           UINT message,
                           WPARAM wparam,
                           LPARAM lparam);

private:

  void draw();

private:

  HINSTANCE m_hInstance; //!< Handle to the instance of the module.
  HWND      m_hWnd;      //!< Handle to the instance of the window.
  HDC       m_hDC;       //!< Handle to the instance of Device Context.
  HGLRC     m_hGlRC;     //!< Handle to the instance of OpenGL rendering context.
  int       m_iWidth;    //!< Window width.
  int       m_iHeight;   //!< Window height.
  bool      m_bQuit;     //!< Indicates whether user want to quit from window.
  IState    m_state;     //!< Current interaction state.

private:

  t_ptr<visu_Scene>        m_scene;   //!< Scene.
  t_ptr<visu_Picker>       m_picker;  //!< Picker.
  t_ptr<visu_CommandQueue> m_queue;   //!< Command queue.
  t_ptr<visu_CommandRepo>  m_cmdRepo; //!< Command repo.

};

}

#endif
