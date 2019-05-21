//-----------------------------------------------------------------------------
// Created on: 15 June 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2014-present, Sergey Slyadnev
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

#ifndef test_BSplineSurface_HeaderFile
#define test_BSplineSurface_HeaderFile

// Tests includes
#include <mobius/test_CaseIDs.h>

// testEngine includes
#include <mobius/testEngine_TestCase.h>

// core includes
#include <mobius/core.h>

namespace mobius {

//! Unit test for B-spline surfaces.
class test_BSplineSurface : public testEngine_TestCase
{
public:

  //! Returns Test Case ID.
  //! \return ID of the Test Case.
  static int ID()
  {
    return CaseID_Geom_BSplineSurface;
  }

  //! Returns filename for the description.
  //! \return filename for the description of the Test Case.
  static std::string DescriptionFn()
  {
    return "test_BSplineSurface";
  }

  //! Returns Test Case description directory.
  //! \return description directory for the Test Case.
  static std::string DescriptionDir()
  {
    return "Geom3D";
  }

  //! Returns pointers to the Test Functions to launch.
  //! \param functions [out] output collection of pointers.
  static void Functions(MobiusTestFunctions& functions)
  {
    functions << &evalInDomain
              << &evalOutDomain
              << &evalJSON1
              << &evalJSON2
              << &evalD1InDomain
              << &evalD1OutDomain
              << &evalD2InDomain
              << &evalD2OutDomain
              << &computeEnergy01
              << &computeEnergy02
              << &computeEnergy03
              << &computeEnergy04
              << &computeEnergy05
              << &computeEnergy06
              << &insertKnot01
              << &exchangeUV01
              << &invertPoint01
              << &invertPoint02
              << &invertPoint03
              << &invertPoint04
              << &invertPoint05
              ;
  }

private:

  static outcome evalInDomain    (const int funcID);
  static outcome evalOutDomain   (const int funcID);
  static outcome evalJSON1       (const int funcID);
  static outcome evalJSON2       (const int funcID);
  static outcome evalD1InDomain  (const int funcID);
  static outcome evalD1OutDomain (const int funcID);
  static outcome evalD2InDomain  (const int funcID);
  static outcome evalD2OutDomain (const int funcID);
  static outcome computeEnergy01 (const int funcID);
  static outcome computeEnergy02 (const int funcID);
  static outcome computeEnergy03 (const int funcID);
  static outcome computeEnergy04 (const int funcID);
  static outcome computeEnergy05 (const int funcID);
  static outcome computeEnergy06 (const int funcID);
  static outcome insertKnot01    (const int funcID);
  static outcome exchangeUV01    (const int funcID);
  static outcome invertPoint01   (const int funcID);
  static outcome invertPoint02   (const int funcID);
  static outcome invertPoint03   (const int funcID);
  static outcome invertPoint04   (const int funcID);
  static outcome invertPoint05   (const int funcID);

};

};

#endif
