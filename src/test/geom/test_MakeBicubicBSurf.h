//-----------------------------------------------------------------------------
// Created on: 14 December 2018
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

#ifndef test_MakeBicubicBSurf_HeaderFile
#define test_MakeBicubicBSurf_HeaderFile

// Test includes
#include <mobius/test_CaseIDs.h>

// TestEngine includes
#include <mobius/testEngine_TestCase.h>

// Geom includes
#include <mobius/geom_BSplineSurface.h>

namespace mobius {

//! Test functions for bicubic surface builder.
class test_MakeBicubicBSurf : public testEngine_TestCase
{
public:

  //! Returns Test Case ID.
  //! \return ID of the Test Case.
  static int ID()
  {
    return CaseID_Geom_MakeBicubicBSurf;
  }

  //! Returns filename for the description.
  //! \return filename for the description of the Test Case.
  static std::string DescriptionFn()
  {
    return "test_MakeBicubicBSurf";
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
    functions << &test001
              << &test002
              << &test003
              << &test004
              << &test005
              << &test006
              << &test007
              << &test008
    ; // Put semicolon here for convenient adding new functions above ;)
  }

private:

  //! This function verifies that the passed B-surface satisfies the given
  //! sixteen constraints in its corner points `(0,0); (0,1); (1,0); (1,1)`.
  static bool verifyConstraints(const t_xyz&          S00,
                                const t_xyz&          S01,
                                const t_xyz&          S10,
                                const t_xyz&          S11,
                                const t_xyz&          dS_du00,
                                const t_xyz&          dS_du01,
                                const t_xyz&          dS_du10,
                                const t_xyz&          dS_du11,
                                const t_xyz&          dS_dv00,
                                const t_xyz&          dS_dv01,
                                const t_xyz&          dS_dv10,
                                const t_xyz&          dS_dv11,
                                const t_xyz&          d2S_dudv00,
                                const t_xyz&          d2S_dudv01,
                                const t_xyz&          d2S_dudv10,
                                const t_xyz&          d2S_dudv11,
                                const t_ptr<t_bsurf>& resSurf);

  //! Performs test logic.
  static outcome performTest(const int    funcID,
                             const t_xyz& S00,
                             const t_xyz& S01,
                             const t_xyz& S10,
                             const t_xyz& S11,
                             const t_xyz& dS_du00,
                             const t_xyz& dS_du01,
                             const t_xyz& dS_du10,
                             const t_xyz& dS_du11,
                             const t_xyz& dS_dv00,
                             const t_xyz& dS_dv01,
                             const t_xyz& dS_dv10,
                             const t_xyz& dS_dv11,
                             const t_xyz& d2S_dudv00,
                             const t_xyz& d2S_dudv01,
                             const t_xyz& d2S_dudv10,
                             const t_xyz& d2S_dudv11);

private:

  static outcome test001(const int funcID);
  static outcome test002(const int funcID);
  static outcome test003(const int funcID);
  static outcome test004(const int funcID);
  static outcome test005(const int funcID);
  static outcome test006(const int funcID);
  static outcome test007(const int funcID);
  static outcome test008(const int funcID);

};

}

#endif
