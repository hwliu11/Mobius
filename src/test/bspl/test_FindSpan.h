//-----------------------------------------------------------------------------
// Created on: 11 June 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

#ifndef QrTest_BasisFindSpan_HeaderFile
#define QrTest_BasisFindSpan_HeaderFile

// QrTest includes
#include <QrTest_CaseIDs.h>

// QrTestLib includes
#include <QrTestLib_TestCase.h>

// QrCore includes
#include <QrCore_Types.h>

//! Unit test for B-spline basis algorithm dedicated to searching of
//! knot intervals.
class QrTest_BasisFindSpan
{
// Construction & destruction:
public:

  QrTest_EXPORT QrTest_BasisFindSpan();
  QrTest_EXPORT virtual ~QrTest_BasisFindSpan();

public:

  //! Returns Test Case ID.
  //! \return ID of the Test Case.
  virtual inline int ID() const
  {
    return CaseID_BSpl_Basis_BasisFindSpan;
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

#endif
