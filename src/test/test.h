//-----------------------------------------------------------------------------
// Created on: 11 June 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

#ifndef test_HeaderFile
#define test_HeaderFile

#if defined _WIN32
  #if defined mobiusTest_EXPORTS
    #define mobiusTest_EXPORT __declspec(dllexport)
  #else
    #define mobiusTest_EXPORT __declspec(dllimport)
  #endif
#else
  #define mobiusTest_EXPORT
#endif

#define test_NotUsed(x)

#endif
