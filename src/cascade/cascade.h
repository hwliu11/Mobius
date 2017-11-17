//-----------------------------------------------------------------------------
// Created on: 10 June 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef cascade_HeaderFile
#define cascade_HeaderFile

#if defined _WIN32
  #if defined mobiusCascade_EXPORTS
    #define mobiusCascade_EXPORT __declspec(dllexport)
  #else
    #define mobiusCascade_EXPORT __declspec(dllimport)
  #endif
#else
  #define mobiusCascade_EXPORT
#endif

#define cascade_NotUsed(x)

#endif
