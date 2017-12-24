//-----------------------------------------------------------------------------
// Created on: 11 June 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

// Tests includes
#include <mobius/test_EffectiveN.h>
#include <mobius/test_EffectiveNDers.h>
#include <mobius/test_FindSpan.h>
#include <mobius/test_KnotMultiset.h>
#include <mobius/test_N.h>
#include <mobius/test_UnifyKnots.h>
#include <mobius/test_InterpolateCurve3D.h>
#include <mobius/test_KnotsAverage.h>
#include <mobius/test_Line3D.h>
#include <mobius/test_ParamsCentripetal.h>
#include <mobius/test_ParamsChordLength.h>
#include <mobius/test_ParamsUniform.h>
#include <mobius/test_PointOnLine.h>
#include <mobius/test_Quaternion.h>

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
  // TODO: fix Test Cases
  //testEngine_Launcher<test_FindSpan>      T1;
  //testEngine_Launcher<test_BasisN>             T2;

  std::vector< core_Ptr<testEngine_CaseLauncherAPI> > CaseLaunchers;
  //
  CaseLaunchers.push_back( new testEngine_CaseLauncher<test_EffectiveN> );
  CaseLaunchers.push_back( new testEngine_CaseLauncher<test_EffectiveNDers> );
  CaseLaunchers.push_back( new testEngine_CaseLauncher<test_KnotMultiset> );
  CaseLaunchers.push_back( new testEngine_CaseLauncher<test_UnifyKnots> );
  CaseLaunchers.push_back( new testEngine_CaseLauncher<test_Quaternion> );
  CaseLaunchers.push_back( new testEngine_CaseLauncher<test_ParamsUniform> );
  CaseLaunchers.push_back( new testEngine_CaseLauncher<test_ParamsChordLength> );
  CaseLaunchers.push_back( new testEngine_CaseLauncher<test_ParamsCentripetal> );
  CaseLaunchers.push_back( new testEngine_CaseLauncher<test_KnotsAverage> );
  CaseLaunchers.push_back( new testEngine_CaseLauncher<test_Line3D> );
  CaseLaunchers.push_back( new testEngine_CaseLauncher<test_InterpolateCurve3D> );
  CaseLaunchers.push_back( new testEngine_CaseLauncher<test_PointOnLine> );

  // Launcher of entire test suite
  testEngine_Launcher Launcher;
  for ( int c = 0; c < (int) CaseLaunchers.size(); ++c )
    Launcher << CaseLaunchers[c];

  PRINT_DECOR
  if ( !Launcher.Launch(&std::cout) ) // Launch Test Cases
  {
    std::cout << "\t***\n\tTests FAILED" << std::endl;
    PRINT_DECOR
    return 1;
  }

  std::cout << "\t***\n\tTests SUCCEEDED" << std::endl;
  PRINT_DECOR
  return 0;
}
