//-----------------------------------------------------------------------------
// Created on: 11 June 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

#ifndef test_FindSpan_HeaderFile
#define test_FindSpan_HeaderFile

// Tests includes
#include <mobius/test_CaseIDs.h>

// testEngine includes
#include <mobius/testEngine_TestCase.h>

// core includes
#include <mobius/core.h>

namespace mobius {

//! Unit test for B-spline basis algorithm dedicated to searching of
//! knot intervals.
class test_FindSpan
{
// Construction & destruction:
public:

  mobiusTest_EXPORT test_FindSpan();
  mobiusTest_EXPORT virtual ~test_FindSpan();

public:

  //! Returns Test Case ID.
  //! \return ID of the Test Case.
  virtual inline int ID() const
  {
    return CaseID_BSpl_FindSpan;
  }

  //! Returns Test Case description.
  //! \return description of the Test Case.
  virtual std::string Description() const
  {
    return "Test suite for the algorithm finding knot spans";
  }

public:

  bool LaunchFunction();

private:

  bool testCase1() const;
  bool testCase2() const;
  bool testCase3() const;

private:

  virtual void beforeAll() {};
  virtual void afterAll() {};

};

};

#endif
