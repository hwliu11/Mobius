//-----------------------------------------------------------------------------
// Created on: 11 June 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

// QrTest includes
#include <QrTest_ADBSplineCurve3D.h>
#include <QrTest_BasisEffectiveN.h>
#include <QrTest_BasisEffectiveNDers.h>
#include <QrTest_BasisFindSpan.h>
#include <QrTest_BasisKnotMultiset.h>
#include <QrTest_BasisN.h>
#include <QrTest_BasisUnifyKnots.h>
#include <QrTest_InterpolateCurve3D.h>
#include <QrTest_KnotsAverage.h>
#include <QrTest_Line3D.h>
#include <QrTest_ParamsCentripetal.h>
#include <QrTest_ParamsChordLength.h>
#include <QrTest_ParamsUniform.h>
#include <QrTest_PointOnLine.h>
#include <QrTest_Quaternion.h>
#include <QrTest_ReadXYZ.h>

// QrTestLib includes
#include <QrTestLib_Launcher.h>

// QrCore includes
#include <QrCore_Types.h>

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

int main(int /*argc*/, char* /*argv[]*/)
{
  // TODO: fix Test Cases
  //QrTestLib_Launcher<QrTest_BasisFindSpan>      T1;
  //QrTestLib_Launcher<QrTest_BasisN>             T2;

  std::vector< QrPtr<QrTestLib_CaseLauncherAPI> > CaseLaunchers;
  CaseLaunchers.push_back( new QrTestLib_CaseLauncher<QrTest_ADBSplineCurve3D> );
  CaseLaunchers.push_back( new QrTestLib_CaseLauncher<QrTest_BasisEffectiveN> );
  CaseLaunchers.push_back( new QrTestLib_CaseLauncher<QrTest_BasisEffectiveNDers> );
  CaseLaunchers.push_back( new QrTestLib_CaseLauncher<QrTest_BasisKnotMultiset> );
  CaseLaunchers.push_back( new QrTestLib_CaseLauncher<QrTest_BasisUnifyKnots> );
  CaseLaunchers.push_back( new QrTestLib_CaseLauncher<QrTest_Quaternion> );
  CaseLaunchers.push_back( new QrTestLib_CaseLauncher<QrTest_ParamsUniform> );
  CaseLaunchers.push_back( new QrTestLib_CaseLauncher<QrTest_ParamsChordLength> );
  CaseLaunchers.push_back( new QrTestLib_CaseLauncher<QrTest_ParamsCentripetal> );
  CaseLaunchers.push_back( new QrTestLib_CaseLauncher<QrTest_KnotsAverage> );
  CaseLaunchers.push_back( new QrTestLib_CaseLauncher<QrTest_Line3D> );
  CaseLaunchers.push_back( new QrTestLib_CaseLauncher<QrTest_InterpolateCurve3D> );
  CaseLaunchers.push_back( new QrTestLib_CaseLauncher<QrTest_PointOnLine> );
  CaseLaunchers.push_back( new QrTestLib_CaseLauncher<QrTest_ReadXYZ> );

  // Launcher of entire test suite
  QrTestLib_Launcher Launcher;
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
