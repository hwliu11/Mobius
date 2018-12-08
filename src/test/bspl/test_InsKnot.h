//-----------------------------------------------------------------------------
// Created on: 07 December 2018
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

#ifndef test_InsKnot_HeaderFile
#define test_InsKnot_HeaderFile

// Tests includes
#include <mobius/test_CaseIDs.h>

// testEngine includes
#include <mobius/testEngine_TestCase.h>

// BSpl includes
#include <mobius/bspl_ParamDirection.h>

// Geom includes
#include <mobius/geom_BSplineCurve.h>
#include <mobius/geom_BSplineSurface.h>

namespace mobius {

//! Unit test for knot insertion algorithm.
class test_InsKnot : public testEngine_TestCase
{
public:

  //! Returns Test Case ID.
  //! \return ID of the Test Case.
  static int ID()
  {
    return CaseID_BSpl_InsKnot;
  }

  //! Returns filename for the description.
  //! \return filename for the description of the Test Case.
  static std::string DescriptionFn()
  {
    return "test_InsKnot";
  }

  //! Returns Test Case description directory.
  //! \return description directory for the Test Case.
  static std::string DescriptionDir()
  {
    return "BSpl";
  }

  //! Returns pointers to the Test Functions to launch.
  //! \param[out] functions output collection of pointers.
  static void Functions(MobiusTestFunctions& functions)
  {
    functions << &test01
              << &test02
              << &test03
              << &test04;
  }

private:

  static bool insertKnotCurve(const ptr<bcurve>&         curve,
                              const double               u,
                              const int                  r,
                              const std::vector<double>& refKnots);

  static bool insertKnotSurface(const ptr<bsurf>&          surface,
                                const double               knot,
                                const bspl_ParamDirection  dir,
                                const int                  r,
                                const std::vector<double>& refKnots);

private:

  //! Test scenario 01 for inserting knots in B-curve.
  //! \param[in] funcID ID of the test function.
  //! \return outcome structure representing the execution result.
  static outcome test01(const int funcID);

  //! Test scenario 02 for inserting knots in B-curve.
  //! \param[in] funcID ID of the test function.
  //! \return outcome structure representing the execution result.
  static outcome test02(const int funcID);

  //! Test scenario 03 for inserting knots in B-surface.
  //! \param[in] funcID ID of the test function.
  //! \return outcome structure representing the execution result.
  static outcome test03(const int funcID);

  //! Test scenario 04 for inserting knots in B-surface.
  //! \param[in] funcID ID of the test function.
  //! \return outcome structure representing the execution result.
  static outcome test04(const int funcID);

};

};

#endif
