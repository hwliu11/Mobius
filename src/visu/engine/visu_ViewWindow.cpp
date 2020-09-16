//-----------------------------------------------------------------------------
// Created on: 14 June 2013
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
#include <mobius/visu_ViewWindow.h>

// visu includes
#include <mobius/visu_ConsoleWindow.h>
#include <mobius/visu_ViewCmd.h>

// GL includes
#include <mobius/ARB_Multisample.h>

#define QrINTERACTION_RotoSensitivityPx 15

//-----------------------------------------------------------------------------
// Viewer
//-----------------------------------------------------------------------------

//! Constructor.
//! \param Queue [in] command queue.
//! \param CmdRepo [in] command repo.
mobius::visu_ViewWindow::visu_ViewWindow(const t_ptr<visu_CommandQueue>& Queue,
                                         const t_ptr<visu_CommandRepo>& CmdRepo)
{
  m_hInstance = NULL;
  m_hWnd      = NULL;
  m_hDC       = NULL;
  m_hGlRC     = NULL;
  m_iWidth    = 0;
  m_iHeight   = 0;
  m_scene     = new visu_Scene(new visu_Camera);
  m_picker    = new visu_Picker(m_scene);
  m_queue     = Queue;
  m_cmdRepo   = CmdRepo;
}

//! Destructor.
mobius::visu_ViewWindow::~visu_ViewWindow()
{
  this->Close();
}

//! Creates new View Window.
//! \param theLeft [in] coordinate of left corner.
//! \param theTop [in] coordinate of top corner.
//! \param theWidth [in] window width.
//! \param theHeight [in] window height.
//! \return true in case of success, false -- otherwise.
bool mobius::visu_ViewWindow::Create(const int theLeft,
                                     const int theTop,
                                     const int theWidth,
                                     const int theHeight)
{
  /* ==============================
   *  Prepare and show View Window
   * ============================== */

  m_iWidth  = theWidth;
  m_iHeight = theHeight;
  m_bQuit   = false;

  // Initialize Camera
  m_scene->Camera()->Init(m_iWidth, m_iHeight, 0.0, 0.0, 1.0, 1.0,
                          visu_Camera::SceneRotoStyle_AroundMobileAxes);

  // Register the window class once
  static HINSTANCE APP_INSTANCE = NULL;
  if ( APP_INSTANCE == NULL )
  {
    APP_INSTANCE = GetModuleHandleW(NULL);
    m_hInstance = APP_INSTANCE;

    WNDCLASSW WC;
    WC.cbClsExtra    = 0;
    WC.cbWndExtra    = 0;
    WC.hbrBackground = (HBRUSH) GetStockObject(BLACK_BRUSH);
    WC.hCursor       = LoadCursor(NULL, IDC_ARROW);
    WC.hIcon         = LoadIcon(NULL, IDI_APPLICATION);
    WC.hInstance     = APP_INSTANCE;
    WC.lpfnWndProc   = (WNDPROC) visu_ViewWindow::wndProcProxy;
    WC.lpszClassName = L"OpenGLClass";
    WC.lpszMenuName  = 0;
    WC.style         = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;

    if ( !RegisterClassW(&WC) )
    {
      FatalAppExit(NULL, "RegisterClass() failed:\nCannot register window class.");
      return false;
    }
  }

  // Set coordinates of for window's area rectangle
  RECT Rect;
  SetRect(&Rect,
          theLeft, theTop,
          theLeft + theWidth, theTop + theHeight);

  // Adjust window rectangle
  AdjustWindowRect(&Rect, WS_OVERLAPPEDWINDOW, false);

  // Create window
  m_hWnd = CreateWindow("OpenGLClass",
                        "Mobius >>> 3D",
                        WS_OVERLAPPEDWINDOW,
                        Rect.left, Rect.top, // Adjusted x, y positions
                        Rect.right - Rect.left, Rect.bottom - Rect.top, // Adjusted width and height
                        NULL, NULL,
                        m_hInstance,
                        this);

  // Check if window has been created successfully
  if ( m_hWnd == NULL )
  {
    FatalAppExit( NULL, TEXT("CreateWindow() failed!") );
    return false;
  }

  /* ==========================
   *  Set pixel format for HDC
   * ========================== */

  // Get device context
  m_hDC = GetDC(m_hWnd);

  // PIXEL FORMAT describes the "qualities" that each pixel in the window will
  // have. There are 3 sub-steps here:
  //
  // A> Create the PFD and set it up to describe the pixel format we DESIRE
  //
  // B> Call ChoosePixelFormat() to make Windows choose us the ID of the
  //    appropriate pixel format that is CLOSEST to desired pixel format
  //
  // C> Call SetPixelFormat() using the integer ID number that
  //    ChoosePixelFormat() returned in step B

  // Create PFD
  PIXELFORMATDESCRIPTOR PFD = {0}; // Start it out with ALL zeroes in ALL of its fields

  // Choose Pixel Format taking into account our desire to have
  // ARB multisampling enabled. This multisampling is achieved by
  // two-pass window creation approach (see NeHe, lesson 46)
  int chosenPixelFormat;
  if ( !arbMultisampleSupported )
  {
    // Set only meaningful properties of PFD
    PFD.nSize = sizeof(PIXELFORMATDESCRIPTOR);
    PFD.nVersion = 1; // Always 1
    PFD.dwFlags = PFD_SUPPORT_OPENGL | // OpenGL support (not DirectDraw)
                  PFD_DOUBLEBUFFER   | // Double buffering support
                  PFD_DRAW_TO_WINDOW;  // Draw to the app window, not to a bitmap image
    PFD.iPixelType = PFD_TYPE_RGBA; // Red, green, blue, alpha for each pixel
    PFD.cColorBits = 24; // 24 bit == 8 bits for red, 8 for green, 8 for blue.
                         // This count of color bits EXCLUDES alpha
    PFD.cDepthBits = 32; // 32 bits to measure pixel depth

    // System might not be able to use the desired pixel format exactly
    // ChoosePixelFormat() function will examine the desired PIXELFORMATDESCRIPTOR
    // structure and select ID for the pixel format that most closely matches
    // the desired pixel format
    chosenPixelFormat = ChoosePixelFormat(m_hDC, &PFD);
    if ( chosenPixelFormat == 0 )
    {
      FatalAppExit( NULL, TEXT("ChoosePixelFormat() failed!") );
    }
  }
  else
    chosenPixelFormat = arbMultisampleFormat;

  // Call SetPixelFormat() using the integer ID number that
  // ChoosePixelFormat() returned previously
  int aResID = SetPixelFormat(m_hDC, chosenPixelFormat, &PFD);
  if ( aResID == 0 )
  {
    FatalAppExit( NULL, TEXT("SetPixelFormat() failed!") );
  }

  /* ==================================
   *  Create rendering context (HGLRC)
   * ================================== */

  // HGLRC is somewhat for OpenGL to draw into. HGLRC must be created
  // compatible with HDC of the window

  m_hGlRC = wglCreateContext(m_hDC);

  // Connect HGLRC with Device Context (HDC) of the window
  wglMakeCurrent(m_hDC, m_hGlRC);

  /* =================================================
   *  Do trick (window re-creating) for multisampling
   * ================================================= */

  // Once our window is created, we want to query what samples are
  // available and we call our InitMuliSample() function (see NeHe
  // approach to multisampling)
#if defined CHECK_FOR_MULTISAMPLE
  if ( !arbMultisampleSupported )
  {
    if ( InitMultisample(APP_INSTANCE, m_hWnd, PFD) )
    {
      this->Close();
      return this->Create(theLeft, theTop, theWidth, theHeight);
    }
  }
#endif

  // Show window finally
  ShowWindow(m_hWnd, TRUE);

  return true;
}

//! Resets scene for its initial state.
void mobius::visu_ViewWindow::ResetScene()
{
  // Calculate characteristic scene size
  double xMin, xMax, yMin, yMax, zMin, zMax;
  m_scene->GetBounds(xMin, xMax, yMin, yMax, zMin, zMax);
  t_xyz P_min(xMin, yMin, zMin), P_max(xMax, yMax, zMax);
  const double diag = (P_max - P_min).Modulus();

  // Provide it to Camera
  m_scene->Camera()->InitSceneSize(diag);

  // Set original position
  m_scene->Camera()->SetPosition(0.0, 0.0, diag);

  // Set original rotation and redraw
  m_scene->Camera()->SetRotation( core_Quaternion() );
}

//! Resizes viewport according to the passed parameters.
//! \param theWidth [in] new width.
//! \param theHeight [in] new height.
void mobius::visu_ViewWindow::Resize(const int theWidth,
                                     const int theHeight)
{
  m_iWidth = theWidth;
  m_iHeight = theHeight;

  // Set up the viewport to be the whole width and height of the client
  // area of the window (the client area excludes the title bar and the
  // maximize/minimize buttons)
  glViewport(0, 0, m_iWidth, m_iHeight);

  // Initialize Camera
  m_scene->Camera()->InitViewportSize(m_iWidth, m_iHeight);
}

//! Switches Camera's rotation style.
void mobius::visu_ViewWindow::SwitchRotationStyle()
{
  visu_Camera::SceneRotoStyle rotoStyle = m_scene->Camera()->RotoStyle();
  if ( rotoStyle == visu_Camera::SceneRotoStyle_AroundFixedAxes )
    rotoStyle = visu_Camera::SceneRotoStyle_AroundMobileAxes;
  else if ( rotoStyle == visu_Camera::SceneRotoStyle_AroundMobileAxes )
    rotoStyle = visu_Camera::SceneRotoStyle_AroundFixedAxes;

  // Set new rotation style
  m_scene->Camera()->InitRotoStyle(rotoStyle);
}

//! Closes View Window.
void mobius::visu_ViewWindow::Close()
{
  if ( m_hWnd == NULL )
    return;

  wglMakeCurrent(NULL, NULL);

  // Delete the rendering context, we no longer need it
  wglDeleteContext(m_hGlRC);
  m_hGlRC = NULL;

  // Release DC
  ReleaseDC(m_hWnd, m_hDC);
  m_hDC = NULL;

  // Destroy window
  DestroyWindow(m_hWnd);
  m_hWnd = NULL;
}

//! Starts message loop.
void mobius::visu_ViewWindow::StartMessageLoop()
{
  MSG Msg;
  while ( !m_bQuit )
  {
    if ( PeekMessage(&Msg, NULL, 0, 0, PM_REMOVE) )
    {
      if ( Msg.message == WM_QUIT )
        break;

      TranslateMessage(&Msg);
      DispatchMessage(&Msg);
    }
    // If there are no pending messages from OS to proceed with,
    // we draw the scene
    else
    {
      // Get next command from queue
      const t_ptr<visu_BaseCmd>& LastCommand = m_queue->Last();

      // Check if this command is for View Window
      if ( !LastCommand.IsNull() )
      {
        // Check command type: we can proceed only with visualization ones
        visu_ViewCmd* CmdPtr = dynamic_cast<visu_ViewCmd*>( LastCommand.Access() );
        if ( CmdPtr )
        {
          // Execute command
          if ( !LastCommand->Execute() )
            visu_ConsoleWindow::DisplayMessage("Viewer", "Command failed!", false);

          // Pop command AFTER execution in order to assure that Console thread
          // will contnue only after Viewer did its job. This is not really
          // important but allows us to avoid stupid mix up in output messages
          m_queue->Pop();
        }
      }

      // Draw using OpenGL: this region is the heart of our view. The most
      // execution time is spent repeating this draw() function
      this->draw();
    }
  }
}

//! Rendering function.
void mobius::visu_ViewWindow::draw()
{
  if ( m_scene.IsNull() )
    FatalAppExit( NULL, TEXT("Scene is not initialized!") );

  // Draw Scene
  m_scene->GL_Draw();

  // Swap buffers
  SwapBuffers(m_hDC);
}

//! Proxy for wndProc() class member.
LRESULT WINAPI mobius::visu_ViewWindow::wndProcProxy(HWND   hwnd,
                                                     UINT   message,
                                                     WPARAM wparam,
                                                     LPARAM lparam)
{
  if ( message == WM_CREATE )
  {
    // Save pointer to our class instance (sent on window create) to window storage
    CREATESTRUCTW* pCreateStruct = (CREATESTRUCTW*) lparam;
    SetWindowLongPtr(hwnd, int (GWLP_USERDATA), (LONG_PTR) pCreateStruct->lpCreateParams);
  }

  // Get pointer to our class instance
  visu_ViewWindow* pThis = (visu_ViewWindow*) GetWindowLongPtr( hwnd, int (GWLP_USERDATA) );
  return (pThis != NULL)? pThis->wndProc(hwnd, message, wparam, lparam)
                        : DefWindowProcW(hwnd, message, wparam, lparam);
}

//! Window procedure.
LRESULT mobius::visu_ViewWindow::wndProc(HWND   hwnd,
                                         UINT   message,
                                         WPARAM wparam,
                                         LPARAM lparam)
{
  switch( message )
  {
    case WM_CREATE:
      return 0;

    case WM_PAINT:
    {
      HDC hdc;
      PAINTSTRUCT ps;
      hdc = BeginPaint(hwnd, &ps);

      // Don't draw here as it will be too slow.
      EndPaint(hwnd, &ps);
    }
    return 0;

    case WM_SIZE:
      switch ( wparam )
      {
        case SIZE_MAXIMIZED:
        case SIZE_RESTORED:
          this->Resize( LOWORD(lparam), HIWORD(lparam) );
          break;
        default: break;
      }
      return 0;

    case WM_KEYDOWN:
      switch ( wparam )
      {
        case VK_ESCAPE:
          PostQuitMessage(0);
          break;
        case VK_LEFT:
          m_scene->Camera()->MoveRight();
          break;
        case VK_UP:
          m_scene->Camera()->MoveDown();
          break;
        case VK_RIGHT:
          m_scene->Camera()->MoveLeft();
          break;
        case VK_DOWN:
          m_scene->Camera()->MoveUp();
          break;
        case 0x4E:
          m_scene->Camera()->MoveNearer();
          break;
        case 0x46:
          m_scene->Camera()->MoveFarther();
          break;
        case 0x52:
          this->ResetScene();
          break;
        case 0x53:
          this->SwitchRotationStyle();
          break;
        case 0x58:
        {
          if ( m_state.IsCtrlPressed )
            m_scene->Camera()->RotateX();
          break;
        }
        case 0x59:
        {
          if ( m_state.IsCtrlPressed )
            m_scene->Camera()->RotateY();
          break;
        }
        case 0x5A:
        {
          if ( m_state.IsCtrlPressed )
            m_scene->Camera()->RotateZ();
          break;
        }
        case VK_CONTROL:
          // Store state
          m_state.IsCtrlPressed = true;
          break;
        default:
          break;
      }
      return 0;

    case WM_KEYUP:
      switch( wparam )
      {
        case VK_CONTROL:
          // Store state
          m_state.IsCtrlPressed = false;
          break;
        default:
          break;
      }
      return 0;

    case WM_MOUSEWHEEL:
    {
      if ( GET_WHEEL_DELTA_WPARAM(wparam) > 0 )
        m_scene->Camera()->MoveNearer();
      else if ( GET_WHEEL_DELTA_WPARAM(wparam) < 0 )
        m_scene->Camera()->MoveFarther();

      return 0;
    }

    case WM_LBUTTONDOWN:
    {
      const int x = LOWORD(lparam);
      const int y = HIWORD(lparam);

      // Get click coordinates in World's referential
      GLdouble X, Y, Z;
      visu_Utils::DisplayToWorld(x, y, X, Y, Z);

      // Override Z
      Z = m_scene->Camera()->Position().Z();

      // Store state
      m_state.IsLButtonClicked = true;
      m_state.MousePickedX = x;
      m_state.MousePickedY = y;
      m_state.WorldXYZ.SetX(X);
      m_state.WorldXYZ.SetY(Y);
      m_state.WorldXYZ.SetZ(Z);

      return 0;
    }

    case WM_LBUTTONUP:
    {
      // Store state
      m_state.IsLButtonClicked = false;
      return 0;
    }

    case WM_RBUTTONDOWN:
    {
      const int x = LOWORD(lparam);
      const int y = HIWORD(lparam);

      // Store state
      m_state.IsRButtonClicked = true;
      m_state.MouseDraggedX = x;
      m_state.MouseDraggedY = y;

      return 0;
    }

    case WM_MBUTTONDOWN:
    {
      const int x = LOWORD(lparam);
      const int y = HIWORD(lparam);

      // TODO: get rid of this stupid invocation
      m_picker->Pick(x, y);

      // Store state
      m_state.IsMButtonClicked = true;

      return 0;
    }

    case WM_RBUTTONUP:
    {
      // Store state
      m_state.IsRButtonClicked = false;
      return 0;
    }

    case WM_MBUTTONUP:
    {
      // Store state
      m_state.IsMButtonClicked = false;
      return 0;
    }

    case WM_MOUSEMOVE:
    {
      const int x = LOWORD(lparam);
      const int y = HIWORD(lparam);

      if ( m_state.IsLButtonClicked )
      {
        const bool xGoes = (abs(x - m_state.MousePickedX) > QrINTERACTION_RotoSensitivityPx);
        const bool yGoes = (abs(y - m_state.MousePickedY) > QrINTERACTION_RotoSensitivityPx);

        if ( xGoes )
        {
          const bool isPositiveRoto = (x - m_state.MouseHoverX) > 0;
          if ( isPositiveRoto == m_state.IsPositiveRotoX ) // Check if to continue
            m_scene->Camera()->RotateX(!isPositiveRoto);
          else
          {
            // Store state and let user move a bit more
            m_state.IsPositiveRotoX = isPositiveRoto;
            m_state.MousePickedX = x;
            m_state.MousePickedY = y;
          }
        }
        if ( yGoes )
        {
          const bool isPositiveRoto = (y - m_state.MouseHoverY) > 0;
          if ( isPositiveRoto == m_state.IsPositiveRotoY ) // Check if to continue
            m_scene->Camera()->RotateY(!isPositiveRoto);
          else
          {
            // Store state and let user move a bit more
            m_state.IsPositiveRotoY = isPositiveRoto;
            m_state.MousePickedX = x;
            m_state.MousePickedY = y;
          }
        }
      }
      if ( m_state.IsRButtonClicked )
      {
        const bool xGoes = (abs(x - m_state.MouseDraggedX) > 0);
        const bool yGoes = (abs(y - m_state.MouseDraggedY) > 0);

        if ( xGoes )
        {
          if ( x > m_state.MouseDraggedX )
            m_scene->Camera()->MoveLeft();
          else
            m_scene->Camera()->MoveRight();
        }
        if ( yGoes )
        {
          if ( y > m_state.MouseDraggedY )
            m_scene->Camera()->MoveUp();
          else
            m_scene->Camera()->MoveDown();
        }

        // Store state
        m_state.MouseDraggedX = x;
        m_state.MouseDraggedY = y;
      }

      // Store state
      m_state.MouseHoverX = x;
      m_state.MouseHoverY = y;

      // TODO: get rid of this stupid invocation
      //m_picker->Pick(x, y);

      return 0;
    }

    case WM_DESTROY:
      m_bQuit = true;
      return 0;
  }
  return DefWindowProc(hwnd, message, wparam, lparam);
}
