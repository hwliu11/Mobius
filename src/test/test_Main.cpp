//-----------------------------------------------------------------------------
// Created on: 11 June 2013
//-----------------------------------------------------------------------------
// Copyright (c) 2017, Sergey Slyadnev
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

// Common facilities
#include <mobius/test_CommonFacilities.h>

// Tests includes
#include <mobius/test_ApproxSurf.h>
#include <mobius/test_BSplineCurve.h>
#include <mobius/test_BSplineSurface.h>
#include <mobius/test_Decompose.h>
#include <mobius/test_EffectiveN.h>
#include <mobius/test_EffectiveNDers.h>
#include <mobius/test_FairCurve.h>
#include <mobius/test_FairSurf.h>
#include <mobius/test_FindSpan.h>
#include <mobius/test_InsKnot.h>
#include <mobius/test_Integral.h>
#include <mobius/test_KnotMultiset.h>
#include <mobius/test_Mesh.h>
#include <mobius/test_N.h>
#include <mobius/test_UnifyKnots.h>
#include <mobius/test_InterpolateCurve.h>
#include <mobius/test_KnotsAverage.h>
#include <mobius/test_KnotsUniform.h>
#include <mobius/test_Line3D.h>
#include <mobius/test_MakeBicubicBSurf.h>
#include <mobius/test_ParamsCentripetal.h>
#include <mobius/test_ParamsChordLength.h>
#include <mobius/test_ParamsUniform.h>
#include <mobius/test_PlaneSurface.h>
#include <mobius/test_PointOnLine.h>
#include <mobius/test_PositionCloud.h>
#include <mobius/test_Quaternion.h>
#include <mobius/test_SkinSurface.h>
#include <mobius/test_SVO.h>
#include <mobius/test_XYZ.h>

// testEngine includes
#include <mobius/testEngine_Launcher.h>

// core includes
#include <mobius/core.h>

#define PAUSE \
  system("pause");

#define RET_OK \
  PAUSE \
  return 0;

#define RET_FAILURE \
  PAUSE \
  return 1;

#define PRINT_DECOR \
  std::cout << "------------------------------------------------------------" << std::endl;

DEFINE_TEST_VARIABLES

using namespace mobius;

int main(int /*argc*/, char* /*argv[]*/)
{
  t_ptr<test_CommonFacilities> cf = test_CommonFacilities::Instance();

  // TODO: fix Test Cases
  //testEngine_Launcher<test_FindSpan>      T1;
  //testEngine_Launcher<test_BasisN>        T2;

  std::vector< core_Ptr<testEngine_CaseLauncherAPI> > CaseLaunchers;

  // BSpl group.
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_FindSpan>           (cf->ProgressNotifier) );
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_InsKnot>            (cf->ProgressNotifier) );
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_EffectiveN>         (cf->ProgressNotifier) );
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_EffectiveNDers>     (cf->ProgressNotifier) );
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_KnotMultiset>       (cf->ProgressNotifier) );
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_KnotsAverage>       (cf->ProgressNotifier) );
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_KnotsUniform>       (cf->ProgressNotifier) );
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_ParamsCentripetal>  (cf->ProgressNotifier) );
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_ParamsChordLength>  (cf->ProgressNotifier) );
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_ParamsUniform>      (cf->ProgressNotifier) );
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_UnifyKnots>         (cf->ProgressNotifier) );
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_Decompose>          (cf->ProgressNotifier) );

  //// Core group.
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_Integral>           (cf->ProgressNotifier) );
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_Quaternion>         (cf->ProgressNotifier) );
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_XYZ>                (cf->ProgressNotifier) );

  //// Geom group.
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_InterpolateCurve>   (cf->ProgressNotifier) );
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_Line3D>             (cf->ProgressNotifier) );
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_MakeBicubicBSurf>   (cf->ProgressNotifier) );
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_PointOnLine>        (cf->ProgressNotifier) );
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_PositionCloud>      (cf->ProgressNotifier) );
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_BSplineCurve>       (cf->ProgressNotifier) );
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_BSplineSurface>     (cf->ProgressNotifier) );
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_PlaneSurface>       (cf->ProgressNotifier) );
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_FairCurve>          (cf->ProgressNotifier) );
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_FairSurf>           (cf->ProgressNotifier) );
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_SkinSurface>        (cf->ProgressNotifier) );
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_ApproxSurf>         (cf->ProgressNotifier) );

  // Poly group.
  CaseLaunchers.push_back( new testEngine_CaseLauncher<test_Mesh>               (cf->ProgressNotifier) );
  //CaseLaunchers.push_back( new testEngine_CaseLauncher<test_SVO>                (cf->ProgressNotifier) );

  // Launcher of entire test suite.
  testEngine_Launcher Launcher;
  for ( int c = 0; c < (int) CaseLaunchers.size(); ++c )
    Launcher << CaseLaunchers[c];

  PRINT_DECOR
  if ( !Launcher.Launch(&std::cout) ) // Launch Test Cases.
  {
    std::cout << "\t***\n\tTests FAILED" << std::endl;
    PRINT_DECOR
    return 1;
  }

  std::cout << "\t***\n\tTests SUCCEEDED" << std::endl;
  PRINT_DECOR
  return 0;
}
