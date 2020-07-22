//-----------------------------------------------------------------------------
// Created on: 18 January 2013
// Author:     Colt "MainRoach" McAnlis (by NeHe)
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

// OpenGL includes
#include <gl/gl.h>
#include <gl/glu.h>

// visu includes
#include <mobius/ARB_Multisample.h>

// Declairations to use
#define WGL_SAMPLE_BUFFERS_ARB 0x2041
#define WGL_SAMPLES_ARB        0x2042

bool arbMultisampleSupported = false;
int arbMultisampleFormat = 0;

// To check if the passed extension is available.
bool WGLisExtensionSupported(const char* extension)
{
  const size_t extlen = strlen(extension);
  const char* supported = NULL;

  // Try to use wglGetExtensionStringARB on current DC, if possible
  PROC wglGetExtString = wglGetProcAddress("wglGetExtensionsStringARB");

  if ( wglGetExtString )
    supported = ( ( char*(__stdcall*)(HDC) ) wglGetExtString ) ( wglGetCurrentDC() );

  // If that failed, try standard OpenGL extensions string
  if ( supported == NULL )
    supported = (char*) glGetString(GL_EXTENSIONS);

  // If that dailed too, then no extensions are supported
  if ( supported == NULL )
    return false;

  // Begin examination at start of string, increment by 1 on false match
  for ( const char* p = supported; ; p++ )
  {
    // Advance p up to the next possible match
    p = strstr(p, extension);

    if ( p == NULL )
      return false; // No match

    // Make sure that match is at the start of the string or that
    // the previous char is a space, or else we could accidentally
    // match "wglFunkywglExtension" with "wglExtension"

    // Also, make sure that the following character is space or NULL,
    // or else "wglExtensionTwo" might match "wglExtension"
    if ( (p == supported || p[-1]==' ') && (p[extlen] == '\0' || p[extlen]==' ') )
      return true; // Match
  }
}

// Used to query the multisample frequencies.
bool InitMultisample(HINSTANCE, HWND hWnd, PIXELFORMATDESCRIPTOR)
{
  // Check if the string exists in WGL
  if ( !WGLisExtensionSupported("WGL_ARB_multisample") )
  {
    arbMultisampleSupported = false;
    return false;
  }

  // Get our Pixel Format
  PFNWGLCHOOSEPIXELFORMATARBPROC wglChoosePixelFormatARB = (PFNWGLCHOOSEPIXELFORMATARBPROC) wglGetProcAddress("wglChoosePixelFormatARB");
  if ( !wglChoosePixelFormatARB )
  {
    arbMultisampleSupported = false;
    return false;
  }

  // Get our current Device Context
  HDC hDC = GetDC(hWnd);

  int pixelFormat;
  int valid;
  UINT numFormats;
  GLfloat fAttributes[] = {0.0f, 0.0f};

  // These attributes are the bits we want to test for.
  // Everything is pretty standard, the only one we want to
  // really docus on is the SAMPLE BUFFERS ARB and WGL SAMPLES.
  // These two are going to do the main testing for whether or not
  // multisampling is supported on this hardware
  int iAttributes[] =
  {
    WGL_DRAW_TO_WINDOW_ARB, GL_TRUE,
    WGL_SUPPORT_OPENGL_ARB, GL_TRUE,
    WGL_ACCELERATION_ARB, WGL_FULL_ACCELERATION_ARB,
    WGL_COLOR_BITS_ARB, 24,
    WGL_ALPHA_BITS_ARB, 8,
    WGL_DEPTH_BITS_ARB, 16,
    WGL_STENCIL_BITS_ARB, 0,
    WGL_DOUBLE_BUFFER_ARB, GL_TRUE,
    WGL_SAMPLE_BUFFERS_ARB, GL_TRUE,
    WGL_SAMPLES_ARB, 2,
    0,0
  };

  // First we check if we can get a Pixel Format for 4 samples
  valid = wglChoosePixelFormatARB(hDC, iAttributes, fAttributes, 1, &pixelFormat, &numFormats);

  // If returned true and our format count is greater than 1
  if ( valid && numFormats >= 1 )
  {
    arbMultisampleSupported = true;
    arbMultisampleFormat = pixelFormat;
    return arbMultisampleSupported;
  }

  // Our Pixel Format with 4 samples failed, so we test for 2 samples
  iAttributes[19] = 2;
  valid = wglChoosePixelFormatARB(hDC, iAttributes, fAttributes, 1, &pixelFormat, &numFormats);
  if ( valid && numFormats >= 1 )
  {
    arbMultisampleSupported = true;
    arbMultisampleFormat = pixelFormat;
    return arbMultisampleSupported;
  }

  return arbMultisampleSupported;
}
