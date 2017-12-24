//-----------------------------------------------------------------------------
// Created on: 22 May 2014
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
#include <mobius/test_PointOnLine.h>

// geom includes
#include <mobius/geom_PointOnLine.h>

//! Test scenario 001.
//! \param funcID [in] function ID.
//! \return true in case of success, false -- otherwise.
bool mobius::test_PointOnLine::test1(const int funcID)
{
  // Line & point properties
  xyz O(0, 0, 0);
  xyz Dir(1, 0, 0);
  xyz P(2, 0, 0);

  // Perform test
  return doTest(funcID, O, Dir, P, 1.0e-2, true);
}

//! Test scenario 002.
//! \param funcID [in] function ID.
//! \return true in case of success, false -- otherwise.
bool mobius::test_PointOnLine::test2(const int funcID)
{
  // Line & point properties
  xyz O(0, 0, 0);
  xyz Dir(1, 0, 0);
  xyz P(-2, 0, 0);

  // Perform test
  return doTest(funcID, O, Dir, P, 1.0e-2, true);
}

//! Test scenario 003.
//! \param funcID [in] function ID.
//! \return true in case of success, false -- otherwise.
bool mobius::test_PointOnLine::test3(const int funcID)
{
  // Line & point properties
  xyz O(0, 0, 0);
  xyz Dir(1, 0, 0);
  xyz P(1, 1, 0);

  // Perform test
  return doTest(funcID, O, Dir, P, 1.0e-2, false);
}

//! Test scenario 004.
//! \param funcID [in] function ID.
//! \return true in case of success, false -- otherwise.
bool mobius::test_PointOnLine::test4(const int funcID)
{
  // Line & point properties
  xyz O(0, 0, 0);
  xyz Dir(0, 1, 0);
  xyz P(-0.1, 0, 0);

  // Perform test
  return doTest(funcID, O, Dir, P, 1.0e-2, false);
}

//! Test scenario 005.
//! \param funcID [in] function ID.
//! \return true in case of success, false -- otherwise.
bool mobius::test_PointOnLine::test5(const int funcID)
{
  // Line & point properties
  xyz O(0, 0, 0);
  xyz Dir(0, 1, 0);
  xyz P(-0.001, 0, 0);

  // Perform test
  return doTest(funcID, O, Dir, P, 1.0e-2, true);
}

//! Test scenario 006.
//! \param funcID [in] function ID.
//! \return true in case of success, false -- otherwise.
bool mobius::test_PointOnLine::test6(const int funcID)
{
  // Line & point properties
  xyz O(0, 0, 0);
  xyz Dir(0, 1, 0);
  xyz P(0, 1, 0);

  // Perform test
  return doTest(funcID, O, Dir, P, 1.0e-2, true);
}

//! Test scenario 007.
//! \param funcID [in] function ID.
//! \return true in case of success, false -- otherwise.
bool mobius::test_PointOnLine::test7(const int funcID)
{
  // Line & point properties
  xyz O(0, 0, 0);
  xyz Dir(0, 1, 0);
  xyz P(0, 0, 0);

  // Perform test
  return doTest(funcID, O, Dir, P, 1.0e-2, true);
}

//! Test scenario 008.
//! \param funcID [in] function ID.
//! \return true in case of success, false -- otherwise.
bool mobius::test_PointOnLine::test8(const int funcID)
{
  // Line & point properties
  xyz O(0, 0, 0);
  xyz Dir(0, 1, 0);
  xyz P(0.005, 100, 0);

  // Perform test
  return doTest(funcID, O, Dir, P, 1.0e-2, true);
}

//-----------------------------------------------------------------------------
// Internals
//-----------------------------------------------------------------------------

//! Performs testing on the given data.
//! \param funcID     [in] function ID.
//! \param LineOri    [in] origin of line.
//! \param LineDir    [in] direction of line.
//! \param P          [in] point to check.
//! \param classiPrec [in] classification precision.
//! \param resultRef  [in] expected result.
//! \return true in case of success, false -- otherwise.
bool mobius::test_PointOnLine::doTest(const int    funcID,
                                      const xyz&   LineOri,
                                      const xyz&   LineDir,
                                      const xyz&   P,
                                      const double classiPrec,
                                      const bool   resultRef)
{
  // Construct line
  core_Ptr<geom_Line> L = new geom_Line(LineOri, LineDir);

  // Classify
  geom_PointOnLine IsPointOnLine;
  const bool result = IsPointOnLine(P, L, classiPrec);

  // Set description variables
  SetVarDescr("Origin",     LineOri,    ID(), funcID);
  SetVarDescr("Direction",  LineDir,    ID(), funcID);
  SetVarDescr("uMax",       DBL_MAX,    ID(), funcID);
  SetVarDescr("Point",      P,          ID(), funcID);
  SetVarDescr("Prec",       classiPrec, ID(), funcID);
  SetVarDescr("Result",     result,     ID(), funcID);
  SetVarDescr("Result_ref", resultRef,  ID(), funcID);

  /* ====================
   *  Verify calculation
   * ==================== */

  if ( result != resultRef )
    return false;

  return true;
}
