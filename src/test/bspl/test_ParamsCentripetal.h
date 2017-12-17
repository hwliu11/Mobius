//-----------------------------------------------------------------------------
// Created on: 16 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

#ifndef QrTest_ParamsCentripetal_HeaderFile
#define QrTest_ParamsCentripetal_HeaderFile

// QrTest includes
#include <QrTest_CaseIDs.h>

// QrTestLib includes
#include <QrTestLib_TestCase.h>

// QrCore includes
#include <QrCore_Types.h>

//! Unit test for centripetal parameterization.
class QrTest_ParamsCentripetal : public QrTestLib_TestCase
{
public:

  //! Returns Test Case ID.
  //! \return ID of the Test Case.
  static int ID()
  {
    return CaseID_BSpl_Reconstruct_ParamsCentripetal;
  }

  //! Returns filename for the description.
  //! \return filename for the description of the Test Case.
  static std::string DescriptionFn()
  {
    return "QrTest_ParamsCentripetal";
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
    functions << &test1
              << &test2;
  }

private:

  static bool test1(const int funcID);
  static bool test2(const int funcID);

// Construction is prohibited:
private:

  QrTest_ParamsCentripetal() {}
  QrTest_ParamsCentripetal(const QrTest_ParamsCentripetal&) {}
  void operator=(const QrTest_ParamsCentripetal&) {}

};

#endif
