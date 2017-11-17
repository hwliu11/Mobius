//-----------------------------------------------------------------------------
// Created on: 04 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

#ifndef QrTest_KnotsAverage_HeaderFile
#define QrTest_KnotsAverage_HeaderFile

// QrTest includes
#include <QrTest_CaseIDs.h>

// QrTestLib includes
#include <QrTestLib_TestCase.h>

// QrCore includes
#include <QrCore_Types.h>

//! Unit test for averaging technique of knots selection.
class QrTest_KnotsAverage : public QrTestLib_TestCase
{
public:

  //! Returns Test Case ID.
  //! \return ID of the Test Case.
  static int ID()
  {
    return CaseID_BSpl_Reconstruct_KnotsAverage;
  }

  //! Returns filename for the description.
  //! \return filename for the description of the Test Case.
  static std::string DescriptionFn()
  {
    return "QrTest_KnotsAverage";
  }

  //! Returns Test Case description directory.
  //! \return description directory for the Test Case.
  static std::string DescriptionDir()
  {
    return "Reconstruct";
  }

  //! Returns pointers to the Test Functions to launch.
  //! \param functions [out] output collection of pointers.
  static void Functions(QrTestFunctions& functions)
  {
    functions << &testCase1_noDerivativeConstraints
              << &testCase1_endDerivativeConstraints;
  }

private:

  static bool testCase1_noDerivativeConstraints(const int funcID);
  static bool testCase1_endDerivativeConstraints(const int funcID);

// Construction is prohibited:
private:

  QrTest_KnotsAverage() {}
  QrTest_KnotsAverage(const QrTest_KnotsAverage&) {}
  void operator=(const QrTest_KnotsAverage&) {}

};

#endif
