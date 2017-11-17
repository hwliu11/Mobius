//-----------------------------------------------------------------------------
// Created on: 23 May 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

#ifndef geom_HeaderFile
#define geom_HeaderFile

#if defined _WIN32
  #if defined mobiusGeom_EXPORTS
    #define mobiusGeom_EXPORT __declspec(dllexport)
  #else
    #define mobiusGeom_EXPORT __declspec(dllimport)
  #endif
#else
  #define mobiusGeom_EXPORT
#endif

#define geom_NotUsed(x)

#endif
