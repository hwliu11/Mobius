//-----------------------------------------------------------------------------
// Created on: 11 June 2013
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

  CaseID_Core_Integral,
  CaseID_Core_Quaternion,

  //---------------------------------------------------------------------------
  // Geom library
  //---------------------------------------------------------------------------

  CaseID_Geom_InterpolateCurve3D,
  CaseID_Geom_Line3D,
  CaseID_Geom_PointOnLine,
  CaseID_Geom_BSplineCurve,
  CaseID_Geom_BSplineSurface,
  CaseID_Geom_FairCurve,
  CaseID_Geom_FairSurf

};

#endif
