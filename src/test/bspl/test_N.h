//-----------------------------------------------------------------------------
// Created on: 14 June 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

#ifndef test_BasisN_HeaderFile
#define test_BasisN_HeaderFile

// Tests includes
#include <mobius/test_CaseIDs.h>

// testEngine includes
#include <mobius/testEngine_TestCase.h>

// core includes
#include <mobius/core.h>

namespace mobius {

//! Unit test for calculation of B-spline functions.
class test_N
{
// Construction & destruction:
public:

  mobiusTest_EXPORT test_N();
  mobiusTest_EXPORT virtual ~test_N();

public:

  //! Returns Test Case ID.
  //! \return ID of the Test Case.
  virtual inline int ID() const
  {
    return CaseID_BSpl_N;
  }

  //! Returns Test Case description.
  //! \return description of the Test Case.
  virtual std::string Description() const
  {
    return "Test suite for the algorithm calculating B-spline basis function";
  }

public:

  bool LaunchFunction();

private:

  bool testCase1() const;
  bool testCase2() const;

private:

  virtual void beforeAll() {};
  virtual void afterAll() {};

};

};

#endif
