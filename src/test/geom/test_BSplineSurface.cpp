//-----------------------------------------------------------------------------
// Created on: 15 June 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2013-present, Sergey Slyadnev
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
#include <mobius/test_BSplineSurface.h>

// geom includes
#include <mobius/geom_BSplineSurface.h>

//! Test scenario 001: evaluate B-surface in its domain.
//! \param funcID [in] function ID.
//! \return true in case of success, false -- otherwise.
bool mobius::test_BSplineSurface::evalInDomain(const int funcID)
{
  /* ======================
   *  Prepare input points
   * ====================== */

  // Control points.
  std::vector< std::vector<xyz> >
    Q = { { xyz(0.0, 0.0, 0.0), xyz(0.0, 1.0, 0.0) },
          { xyz(1.0, 0.0, 0.0), xyz(1.0, 1.0, 0.0) },
          { xyz(2.0, 0.0, 0.0), xyz(2.0, 1.0, 0.0) } };

  // Knot vectors.
  const std::vector<double> U = {0, 0, 0.5, 1, 1};
  const std::vector<double> V = {0, 0, 1, 1};

  // Degrees.
  const int p = 1;
  const int q = 1;

  // Construct B-surface.
  core_Ptr<bsurf> surf = new bsurf(Q, U, V, p, q);

  /* ==============
   *  Perform test
   * ============== */

  const double eps = 1e-7;
  const double u   = 0.5;
  const double v   = 0.5;
  //
  const xyz P_ref(1, 0.5, 0);

  // Evaluate.
  xyz P;
  surf->Eval(u, v, P);

  // Check.
  if ( (P - P_ref).Modulus() > eps  )
    return false;

  return true;
}

//! Test scenario 001: evaluate B-surface out of its domain.
//! \param funcID [in] function ID.
//! \return true in case of success, false -- otherwise.
bool mobius::test_BSplineSurface::evalOutDomain(const int funcID)
{
  /* ======================
   *  Prepare input points
   * ====================== */

  // Control points.
  std::vector< std::vector<xyz> >
    Q = { { xyz(0.0, 0.0, 0.0), xyz(0.0, 1.0, 0.0) },
          { xyz(1.0, 0.0, 0.0), xyz(1.0, 1.0, 0.0) },
          { xyz(2.0, 0.0, 0.0), xyz(2.0, 1.0, 0.0) } };

  // Knot vectors.
  const std::vector<double> U = {0, 0, 0.5, 1, 1};
  const std::vector<double> V = {0, 0, 1, 1};

  // Degrees.
  const int p = 1;
  const int q = 1;

  // Construct B-surface.
  core_Ptr<bsurf> surf = new bsurf(Q, U, V, p, q);

  /* ==============
   *  Perform test
   * ============== */

  const double eps = 1e-7;
  const double u   = 1.5;
  const double v   = 1.5;
  //
  const xyz P_ref(3, 1.5, 0);

  // Evaluate.
  xyz P;
  surf->Eval(u, v, P);

  // Check.
  if ( (P - P_ref).Modulus() > eps  )
    return false;

  return true;
}
