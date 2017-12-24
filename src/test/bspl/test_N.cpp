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
#include <mobius/test_N.h>

// bspl includes
#include <mobius/bspl_N.h>

//! Constructor.
mobius::test_N::test_N()
{
}

//! Destructor.
mobius::test_N::~test_N()
{
}

//! Entry point.
//! \return true in case of success, false -- otherwise.
bool mobius::test_N::LaunchFunction()
{
  if ( !this->testCase1() )
    return false;

  if ( !this->testCase2() )
    return false;

  return true;
}

//! Test scenario 001.
//! \return true in case of success, false -- otherwise.
bool mobius::test_N::testCase1() const
{
  const std::vector<double> U = {0.0, 0.2, 0.5, 1.0};
  const int p = 0;

  bspl_N N;

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
bool mobius::test_N::testCase2() const
{
  const std::vector<double> U = {0.0, 0.0, 0.2, 0.5, 1.0, 1.0};
  const int p = 1;

  bspl_N N;

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
