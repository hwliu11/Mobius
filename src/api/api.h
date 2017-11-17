//-----------------------------------------------------------------------------
// Created on: 06 September 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
//-----------------------------------------------------------------------------

#ifndef QrAPI_HeaderFile
#define QrAPI_HeaderFile

// QrCore includes
#include <QrCore_Types.h>

#if defined _WIN32
  #if defined QrAPI_LIB
    #define QrAPI_EXPORT __declspec(dllexport)
  #else
    #define QrAPI_EXPORT __declspec(dllimport)
  #endif
#else
  #define QrAPI_EXPORT
#endif

#define QrAPI_NotUsed(x)

#endif
