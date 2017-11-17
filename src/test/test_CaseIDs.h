//-----------------------------------------------------------------------------
// Created on: 11 June 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

#ifndef QrTest_CaseIDs_HeaderFile
#define QrTest_CaseIDs_HeaderFile

#include <QrTest.h>

//! IDs for Test Cases.
enum QrTest_CaseID
{
  //---------------------------------------------------------------------------
  // Core library
  //---------------------------------------------------------------------------

  CaseID_Core_Quaternion = 1,

  //---------------------------------------------------------------------------
  // BSpl library
  //---------------------------------------------------------------------------

  CaseID_BSpl_Basis_BasisFindSpan,
  CaseID_BSpl_Basis_BasisN,
  CaseID_BSpl_Basis_BasisEffectiveN,
  CaseID_BSpl_Basis_BasisEffectiveNDers,
  CaseID_BSpl_Basis_BasisKnotMultiset,
  CaseID_BSpl_Basis_BasisUnifyKnots,

  CaseID_BSpl_Reconstruct_ParamsUniform,
  CaseID_BSpl_Reconstruct_ParamsChordLength,
  CaseID_BSpl_Reconstruct_ParamsCentripetal,
  CaseID_BSpl_Reconstruct_KnotsAverage,

  //---------------------------------------------------------------------------
  // Geom3D library
  //---------------------------------------------------------------------------

  CaseID_Geom3D_InterpolateCurve,
  CaseID_Geom3D_Line3D,
  CaseID_Geom3D_PointOnLine,
  CaseID_Geom3D_AD_BSplineCurve3D,

  //---------------------------------------------------------------------------
  // Tools library
  //---------------------------------------------------------------------------

  CaseID_Tools_ReadXYZ

};

#endif
