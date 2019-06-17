//-----------------------------------------------------------------------------
// Created on: 17 June 2019
//-----------------------------------------------------------------------------
// Copyright (c) 2019-present, Sergey Slyadnev
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

#ifndef test_ApproxSurf_HeaderFile
#define test_ApproxSurf_HeaderFile

// Test includes
#include <mobius/test_CaseIDs.h>

// TestEngine includes
#include <mobius/testEngine_TestCase.h>

namespace mobius {

//! Test functions for surface approximation.
class test_ApproxSurf : public testEngine_TestCase
{
public:

  //! Returns Test Case ID.
  //! \return ID of the Test Case.
  static int ID()
  {
    return CaseID_Geom_ApproxSurf;
  }

  //! Returns filename for the description.
  //! \return filename for the description of the Test Case.
  static std::string DescriptionFn()
  {
    return "test_ApproxSurf";
  }

  //! Returns Test Case description directory.
  //! \return description directory for the Test Case.
  static std::string DescriptionDir()
  {
    return "editing";
  }

  //! Returns pointers to the Test Functions to launch.
  //! \param[out] functions output collection of pointers.
  static void Functions(MobiusTestFunctions& functions)
  {
    functions << &testApprox01
    ; // Put semicolon here for convenient adding new functions above ;)
  }

private:

  static outcome testApprox01 (const int funcID);

};

};

#endif
