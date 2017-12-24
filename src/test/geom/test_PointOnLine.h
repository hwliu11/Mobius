//-----------------------------------------------------------------------------
// Created on: 22 May 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

#ifndef test_PointOnLine_HeaderFile
#define test_PointOnLine_HeaderFile

// Tests includes
#include <mobius/test_CaseIDs.h>

// testEngine includes
#include <mobius/testEngine_TestCase.h>

// core includes
#include <mobius/core.h>

namespace mobius {

//! Unit test for point-line classifier.
class test_PointOnLine : public testEngine_TestCase
{
public:

  //! Returns Test Case ID.
  //! \return ID of the Test Case.
  static int ID()
  {
    return CaseID_Geom_PointOnLine;
  }

  //! Returns filename for the description.
  //! \return filename for the description of the Test Case.
  static std::string DescriptionFn()
  {
    return "test_PointOnLine";
  }

  //! Returns Test Case description directory.
  //! \return description directory for the Test Case.
  static std::string DescriptionDir()
  {
    return "Geom3D";
  }

  //! Returns pointers to the Test Functions to launch.
  //! \param functions [out] output collection of pointers.
  static void Functions(MobiusTestFunctions& functions)
  {
    functions << &test1
              << &test2
              << &test3
              << &test4
              << &test5
              << &test6
              << &test7
              << &test8;
  }

private:

  static bool test1(const int funcID);
  static bool test2(const int funcID);
  static bool test3(const int funcID);
  static bool test4(const int funcID);
  static bool test5(const int funcID);
  static bool test6(const int funcID);
  static bool test7(const int funcID);
  static bool test8(const int funcID);

private:

  static bool doTest(const int    funcID,
                     const xyz&   LineOri,
                     const xyz&   LineDir,
                     const xyz&   P,
                     const double classiPrec,
                     const bool   resultRef);

};

};

#endif
