//-----------------------------------------------------------------------------
// Created on: 11 June 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

// Own include
#include <mobius/test_FindSpan.h>

// bspl includes
#include <mobius/bspl_FindSpan.h>

//! Constructor.
mobius::test_FindSpan::test_FindSpan()
{
}

//! Destructor.
mobius::test_FindSpan::~test_FindSpan()
{
}

//! Entry point.
//! \return true in case of success, false -- otherwise.
bool mobius::test_FindSpan::LaunchFunction()
{
  if ( !this->testCase1() )
    return false;

  if ( !this->testCase2() )
    return false;

  if ( !this->testCase3() )
    return false;

  return true;
}

//! Test scenario 001.
//! \return true in case of success, false -- otherwise.
bool mobius::test_FindSpan::testCase1() const
{
  const std::vector<double> U = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
  const int p = 1;
  const double u1  = 0.0;
  const double u2  = 0.5;
  const double u3  = 1.0;
  const double u4  = 1.1;
  const double u5  = 1.9;
  const double u6  = 2.0;
  const double u7  = 2.7;
  const double u8  = 3.0;
  const double u9  = 3.5;
  const double u10 = 4.5;
  const double u11 = 5.0;
  const double u12 = 100.0;

  bspl_FindSpan FindSpan(U, p);
  int I1  = FindSpan(u1);
  int I2  = FindSpan(u2);
  int I3  = FindSpan(u3);
  int I4  = FindSpan(u4);
  int I5  = FindSpan(u5);
  int I6  = FindSpan(u6);
  int I7  = FindSpan(u7);
  int I8  = FindSpan(u8);
  int I9  = FindSpan(u9);
  int I10 = FindSpan(u10);
  int I11 = FindSpan(u11);
  int I12 = FindSpan(u12);

  return I1 == 0 && I2 == 0 && I3 == 1 && I4 == 1 && I5 == 1 && I6 == 2 &&
         I7 == 2 && I8 == 3 && I9 == 3 && I10 == 4 && I11 == 5 && I12 == 5;
}

//! Test scenario 002.
//! \return true in case of success, false -- otherwise.
bool mobius::test_FindSpan::testCase2() const
{
  const std::vector<double> U = {0.0, 0.0, 1.0, 2.0, 2.0};
  const int p = 1;
  const double u1 = 0.0;
  const double u2 = 0.5;
  const double u3 = 1.0;
  const double u4 = 1.5;
  const double u5 = 2.0;
  const double u6 = 2.5;

  bspl_FindSpan FindSpan(U, p);
  int I1 = FindSpan(u1);
  int I2 = FindSpan(u2);
  int I3 = FindSpan(u3);
  int I4 = FindSpan(u4);
  int I5 = FindSpan(u5);
  int I6 = FindSpan(u6);

  return I1 == 1 && I2 == 1 && I3 == 2 && I4 == 2 && I5 == 4 && I6 == 4;
}

//! Test scenario 003.
//! \return true in case of success, false -- otherwise.
bool mobius::test_FindSpan::testCase3() const
{
  const std::vector<double> U = {0.0, 0.0, 0.0, 1, 2, 3, 4, 4, 5, 5, 5};
  const int p = 1;
  const double u = 5.0;

  bspl_FindSpan FindSpan(U, p);
  int I = FindSpan(u);

  return I == 5;
}
