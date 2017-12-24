//-----------------------------------------------------------------------------
// Created on: 11 June 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

#ifndef test_CaseIDs_HeaderFile
#define test_CaseIDs_HeaderFile

// Tests includes
#include <mobius/test.h>

//! IDs for Test Cases.
enum test_CaseID
{
  //---------------------------------------------------------------------------
  // BSpl library
  //---------------------------------------------------------------------------

  CaseID_BSpl_EffectiveN,
  CaseID_BSpl_EffectiveNDers,
  CaseID_BSpl_FindSpan,
  CaseID_BSpl_KnotMultiset,
  CaseID_BSpl_KnotsAverage,
  CaseID_BSpl_N,
  CaseID_BSpl_ParamsCentripetal,
  CaseID_BSpl_ParamsChordLength,
  CaseID_BSpl_ParamsUniform,
  CaseID_BSpl_UnifyKnots,

  //---------------------------------------------------------------------------
  // Core library
  //---------------------------------------------------------------------------

  CaseID_Core_Quaternion,

  //---------------------------------------------------------------------------
  // Geom library
  //---------------------------------------------------------------------------

  CaseID_Geom_InterpolateCurve3D,
  CaseID_Geom_Line3D,
  CaseID_Geom_PointOnLine

};

#endif
