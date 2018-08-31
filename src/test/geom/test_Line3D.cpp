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
#include <mobius/test_Line3D.h>

// geom includes
#include <mobius/geom_Line.h>

//! Test scenario 001.
//! \param funcID [in] function ID.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_Line3D::test1(const int funcID)
{
  outcome res( DescriptionFn() );

  // Line properties
  core_XYZ O(1, 1, 1);
  core_XYZ Dir(1, 2, 3);

  // Set description variables
  SetVarDescr("Origin",    O,   ID(), funcID);
  SetVarDescr("Direction", Dir, ID(), funcID);

  // Construct line
  core_Ptr<geom_Line> L = new geom_Line(O, Dir);

  // Calculate bounding box
  adouble xMin, yMin, zMin, xMax, yMax, zMax;
  L->Bounds(xMin, xMax, yMin, yMax, zMin, zMax);

  // Evaluate line
  core_XYZ P[3];
  L->Eval( 0.0, P[0]);
  L->Eval( 1.0, P[1]);
  L->Eval(-1.0, P[2]);

  // Referential results
  core_XYZ P_ref[] =
  { core_XYZ(1, 1, 1),
    core_XYZ(1.2672612, 1.5345224, 1.8017837),
    core_XYZ(0.7327387, 0.4654775, 0.1982162) };

  /* ====================================
   *  Verify calculation of bounding box
   * ==================================== */

  const adouble prec = 1.0e-6;

  if ( xMin != -DBL_MAX )
    return res.failure();
  if ( yMin != -DBL_MAX )
    return res.failure();
  if ( zMin != -DBL_MAX )
    return res.failure();
  if ( xMax != DBL_MAX )
    return res.failure();
  if ( yMax != DBL_MAX )
    return res.failure();
  if ( zMax != DBL_MAX )
    return res.failure();

  /* ===========================
   *  Verify evaluation results
   * =========================== */

  size_t num_pts = sizeof(P)/sizeof(core_XYZ);
  for ( size_t pt_idx = 0; pt_idx < num_pts; ++pt_idx )
  {
    const adouble dist = (P[pt_idx] - P_ref[pt_idx]).Modulus();
    if ( dist > prec )
      return res.failure();
  }

  return res.success();
}
