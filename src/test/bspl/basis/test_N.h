//-----------------------------------------------------------------------------
// Created on: 14 June 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

#ifndef QrTest_BasisN_HeaderFile
#define QrTest_BasisN_HeaderFile

// QrTest includes
#include <QrTest_CaseIDs.h>

// QrTestLib includes
#include <QrTestLib_TestCase.h>

// QrCore includes
#include <QrCore_Types.h>

//! Unit test for calculation of B-spline functions.
class QrTest_BasisN
{
// Construction & destruction:
public:

  QrTest_EXPORT QrTest_BasisN();
  QrTest_EXPORT virtual ~QrTest_BasisN();

public:

  //! Returns Test Case ID.
  //! \return ID of the Test Case.
  virtual inline int ID() const
  {
    return CaseID_BSpl_Basis_BasisN;
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

#endif
