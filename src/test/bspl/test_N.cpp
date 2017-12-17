//-----------------------------------------------------------------------------
// Created on: 14 June 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

// Own include
#include <QrTest_BasisN.h>

// QrBSpl includes
#include <QrBSpl_BasisN.h>

//! Constructor.
QrTest_BasisN::QrTest_BasisN()
{
}

//! Destructor.
QrTest_BasisN::~QrTest_BasisN()
{
}

//! Entry point.
//! \return true in case of success, false -- otherwise.
bool QrTest_BasisN::LaunchFunction()
{
  if ( !this->testCase1() )
    return false;

  if ( !this->testCase2() )
    return false;

  return true;
}

//! Test scenario 001.
//! \return true in case of success, false -- otherwise.
bool QrTest_BasisN::testCase1() const
{
  const std::vector<double> U = {0.0, 0.2, 0.5, 1.0};
  const int p = 0;

  QrBSpl_BasisN<double> N;

  double val_0_0 = N(0.0, U, p, 0);
  double val_0_1 = N(0.0, U, p, 1);
  double val_0_2 = N(0.0, U, p, 2);

  double val_1_0 = N(0.1, U, p, 0);
  double val_1_1 = N(0.1, U, p, 1);
  double val_1_2 = N(0.1, U, p, 2);

  double val_2_0 = N(0.3, U, p, 0);
  double val_2_1 = N(0.3, U, p, 1);
  double val_2_2 = N(0.3, U, p, 2);

  double val_3_0 = N(0.6, U, p, 0);
  double val_3_1 = N(0.6, U, p, 1);
  double val_3_2 = N(0.6, U, p, 2);

  // TODO: remove this stuff
  std::cout << val_0_0 << " " << val_0_1 << " " << val_0_2 << std::endl;
  std::cout << val_1_0 << " " << val_1_1 << " " << val_1_2 << std::endl;
  std::cout << val_2_0 << " " << val_2_1 << " " << val_2_2 << std::endl;
  std::cout << val_3_0 << " " << val_3_1 << " " << val_3_2 << std::endl;

  return true;
}

//! Test scenario 002.
//! \return true in case of success, false -- otherwise.
bool QrTest_BasisN::testCase2() const
{
  const std::vector<double> U = {0.0, 0.0, 0.2, 0.5, 1.0, 1.0};
  const int p = 1;

  QrBSpl_BasisN<double> N;

  double val_0_0 = N(0.0, U, p, 0);
  double val_0_1 = N(0.0, U, p, 1);
  double val_0_2 = N(0.0, U, p, 2);
  double val_0_3 = N(0.0, U, p, 3);

  double val_1_0 = N(0.1, U, p, 0);
  double val_1_1 = N(0.1, U, p, 1);
  double val_1_2 = N(0.1, U, p, 2);
  double val_1_3 = N(0.1, U, p, 3);

  double val_2_0 = N(0.3, U, p, 0);
  double val_2_1 = N(0.3, U, p, 1);
  double val_2_2 = N(0.3, U, p, 2);
  double val_2_3 = N(0.3, U, p, 3);

  double val_3_0 = N(0.6, U, p, 0);
  double val_3_1 = N(0.6, U, p, 1);
  double val_3_2 = N(0.6, U, p, 2);
  double val_3_3 = N(0.6, U, p, 3);

  // TODO: remove this stuff
  std::cout << val_0_0 << " " << val_0_1 << " " << val_0_2 << " " << val_0_3 << std::endl;
  std::cout << val_1_0 << " " << val_1_1 << " " << val_1_2 << " " << val_1_3 << std::endl;
  std::cout << val_2_0 << " " << val_2_1 << " " << val_2_2 << " " << val_2_3 << std::endl;
  std::cout << val_3_0 << " " << val_3_1 << " " << val_3_2 << " " << val_3_3 << std::endl;

  return false;
}
