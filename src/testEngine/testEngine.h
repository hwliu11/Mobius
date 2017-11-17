//-----------------------------------------------------------------------------
// Created on: 23 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef testEngine_HeaderFile
#define testEngine_HeaderFile

#if defined _WIN32
  #if defined mobiusTestEngine_EXPORTS
    #define mobiusTestEngine_EXPORT __declspec(dllexport)
  #else
    #define mobiusTestEngine_EXPORT __declspec(dllimport)
  #endif
#else
  #define mobiusTestEngine_EXPORT
#endif

#define testEngine_NotUsed(x)

#endif
