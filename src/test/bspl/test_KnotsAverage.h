//-----------------------------------------------------------------------------
// Created on: 04 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

#ifndef test_KnotsAverage_HeaderFile
#define test_KnotsAverage_HeaderFile

// Tests includes
#include <mobius/test_CaseIDs.h>

// testEngine includes
#include <mobius/testEngine_TestCase.h>

// core includes
#include <mobius/core.h>

namespace mobius {

//! Unit test for averaging technique of knots selection.
class test_KnotsAverage : public testEngine_TestCase
{
public:

  //! Returns Test Case ID.
  //! \return ID of the Test Case.
  static int ID()
  {
    return CaseID_BSpl_KnotsAverage;
  }

  //! Returns filename for the description.
  //! \return filename for the description of the Test Case.
  static std::string DescriptionFn()
  {
    return "test_KnotsAverage";
  }

  //! Returns Test Case description directory.
  //! \return description directory for the Test Case.
  static std::string DescriptionDir()
  {
    return "Reconstruct";
  }

  //! Returns pointers to the Test Functions to launch.
  //! \param functions [out] output collection of pointers.
  static void Functions(MobiusTestFunctions& functions)
  {
    functions << &testCase1_noDerivativeConstraints
              << &testCase1_endDerivativeConstraints;
  }

private:

  static bool testCase1_noDerivativeConstraints(const int funcID);
  static bool testCase1_endDerivativeConstraints(const int funcID);

// Construction is prohibited:
private:

  test_KnotsAverage() {}
  test_KnotsAverage(const test_KnotsAverage&) {}
  void operator=(const test_KnotsAverage&) {}

};

};

#endif
