//-----------------------------------------------------------------------------
// Created on: 11 June 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

#ifndef QrTest_HeaderFile
#define QrTest_HeaderFile

#if defined _WIN32
  #if defined QrTest_LIB
    #define QrTest_EXPORT __declspec(dllexport)
  #else
    #define QrTest_EXPORT __declspec(dllimport)
  #endif
#else
  #define QrTest_EXPORT
#endif

#define QrTest_NotUsed(x)

#endif
