//-----------------------------------------------------------------------------
// Created on: 26 July 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2018-present, Sergey Slyadnev
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

#ifndef test_Integral_HeaderFile
#define test_Integral_HeaderFile

// Tests includes
#include <mobius/test_CaseIDs.h>

// testEngine includes
#include <mobius/testEngine_TestCase.h>

// core includes
#include <mobius/core.h>

namespace mobius {

//! Unit tests for numerical integration.
class test_Integral : public testEngine_TestCase
{
public:

  //! Returns Test Case ID.
  //! \return ID of the Test Case.
  static int ID()
  {
    return CaseID_Core_Integral;
  }

  //! Returns filename for the description.
  //! \return filename for the description of the Test Case.
  static std::string DescriptionFn()
  {
    return "test_Integral";
  }

  //! Returns Test Case description directory.
  //! \return description directory for the Test Case.
  static std::string DescriptionDir()
  {
    return "Core";
  }

  //! Returns pointers to the Test Functions to launch.
  //! \param functions [out] output collection of pointers.
  static void Functions(MobiusTestFunctions& functions)
  {
    functions << &test01
              << &test02
              << &test03
              << &test04
              << &test05
              << &test06
              << &test07
              << &test08
              << &test09
              << &test10
              << &test11;
  }

private:

  static outcome test01 (const int funcID);
  static outcome test02 (const int funcID);
  static outcome test03 (const int funcID);
  static outcome test04 (const int funcID);
  static outcome test05 (const int funcID);
  static outcome test06 (const int funcID);
  static outcome test07 (const int funcID);
  static outcome test08 (const int funcID);
  static outcome test09 (const int funcID);
  static outcome test10 (const int funcID);
  static outcome test11 (const int funcID);

};

};

#endif
