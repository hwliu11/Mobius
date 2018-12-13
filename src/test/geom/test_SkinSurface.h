//-----------------------------------------------------------------------------
// Created on: 13 December 2018
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

#ifndef test_SkinSurface_HeaderFile
#define test_SkinSurface_HeaderFile

// Test includes
#include <mobius/test_CaseIDs.h>

// TestEngine includes
#include <mobius/testEngine_TestCase.h>

// Geom includes
#include <mobius/geom_BSplineSurface.h>

namespace mobius {

//! Test functions for surface skinning.
class test_SkinSurface : public testEngine_TestCase
{
public:

  //! Returns Test Case ID.
  //! \return ID of the Test Case.
  static int ID()
  {
    return CaseID_Geom_SkinSurface;
  }

  //! Returns filename for the description.
  //! \return filename for the description of the Test Case.
  static std::string DescriptionFn()
  {
    return "test_SkinSurface";
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
    ; // Put semicolon here for convenient adding new functions above ;)
  }

private:

  //! Performs test on surface skinning. This function accepts an array of
  //! section curves and builds a skinned surface of a prescribed degree.
  //! To validate the result, the constructed B-surface is compared with
  //! the passed reference surface with a certain tolerance to compensate
  //! any possible round-off errors.
  //!
  //! \param[in] sections array of section curves.
  //! \param[in] D1lead   tangency constraints for the leading curve.
  //! \param[in] D1tail   tangency constraints for the trailing curve.
  //! \param[in] unify    indicates whether to unify section curves or not.
  //!                     If not, it means that the section curves have
  //!                     already been unified.
  //! \param[in] degV     degree in skinning direction to use.
  //! \param[in] surfRef  reference surface to verify the result.
  //!
  //! \return true in case of success, false -- otherwise.
  static bool runtest(const std::vector< ptr<bcurve> >& sections,
                      const std::vector<xyz>&           D1lead,
                      const std::vector<xyz>&           D1tail,
                      const bool                        unify,
                      const int                         degV,
                      const ptr<bsurf>&                 surfRef);

private:

  static outcome test001(const int funcID);
  static outcome test002(const int funcID);
  static outcome test003(const int funcID);

};

};

#endif
