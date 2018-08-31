//-----------------------------------------------------------------------------
// Created on: 14 June 2013
//-----------------------------------------------------------------------------
// Copyright (c) 2017, Sergey Slyadnev
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//    * Neither the name of Sergey Slyadnev nor the
//      names of all contributors may be used to endorse or promote products
//      derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
  if ( !this->testCase1().ok )
    return false;

  if ( !this->testCase2().ok )
    return false;

  return true;
}

//! Test scenario 001.
//! \return true in case of success, false -- otherwise.
mobius::outcome mobius::test_N::testCase1() const
{
  outcome res;//( DescriptionFn() );

  const std::vector<adouble> U = {0.0, 0.2, 0.5, 1.0};
  const int p = 0;

  bspl_N N;

  adouble val_0_0 = N(0.0, U, p, 0);
  adouble val_0_1 = N(0.0, U, p, 1);
  adouble val_0_2 = N(0.0, U, p, 2);

  adouble val_1_0 = N(0.1, U, p, 0);
  adouble val_1_1 = N(0.1, U, p, 1);
  adouble val_1_2 = N(0.1, U, p, 2);

  adouble val_2_0 = N(0.3, U, p, 0);
  adouble val_2_1 = N(0.3, U, p, 1);
  adouble val_2_2 = N(0.3, U, p, 2);

  adouble val_3_0 = N(0.6, U, p, 0);
  adouble val_3_1 = N(0.6, U, p, 1);
  adouble val_3_2 = N(0.6, U, p, 2);

  // TODO: remove this stuff
  std::cout << val_0_0 << " " << val_0_1 << " " << val_0_2 << std::endl;
  std::cout << val_1_0 << " " << val_1_1 << " " << val_1_2 << std::endl;
  std::cout << val_2_0 << " " << val_2_1 << " " << val_2_2 << std::endl;
  std::cout << val_3_0 << " " << val_3_1 << " " << val_3_2 << std::endl;

  return res.success();
}

//! Test scenario 002.
//! \return true in case of success, false -- otherwise.
mobius::outcome mobius::test_N::testCase2() const
{
  outcome res;//( DescriptionFn() );

  const std::vector<adouble> U = {0.0, 0.0, 0.2, 0.5, 1.0, 1.0};
  const int p = 1;

  bspl_N N;

  adouble val_0_0 = N(0.0, U, p, 0);
  adouble val_0_1 = N(0.0, U, p, 1);
  adouble val_0_2 = N(0.0, U, p, 2);
  adouble val_0_3 = N(0.0, U, p, 3);

  adouble val_1_0 = N(0.1, U, p, 0);
  adouble val_1_1 = N(0.1, U, p, 1);
  adouble val_1_2 = N(0.1, U, p, 2);
  adouble val_1_3 = N(0.1, U, p, 3);

  adouble val_2_0 = N(0.3, U, p, 0);
  adouble val_2_1 = N(0.3, U, p, 1);
  adouble val_2_2 = N(0.3, U, p, 2);
  adouble val_2_3 = N(0.3, U, p, 3);

  adouble val_3_0 = N(0.6, U, p, 0);
  adouble val_3_1 = N(0.6, U, p, 1);
  adouble val_3_2 = N(0.6, U, p, 2);
  adouble val_3_3 = N(0.6, U, p, 3);

  // TODO: remove this stuff
  std::cout << val_0_0 << " " << val_0_1 << " " << val_0_2 << " " << val_0_3 << std::endl;
  std::cout << val_1_0 << " " << val_1_1 << " " << val_1_2 << " " << val_1_3 << std::endl;
  std::cout << val_2_0 << " " << val_2_1 << " " << val_2_2 << " " << val_2_3 << std::endl;
  std::cout << val_3_0 << " " << val_3_1 << " " << val_3_2 << " " << val_3_3 << std::endl;

  return res.success();
}
