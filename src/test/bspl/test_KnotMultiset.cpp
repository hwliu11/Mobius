//-----------------------------------------------------------------------------
// Created on: 06 March 2015
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
#include <mobius/test_KnotMultiset.h>

// core includes
#include <mobius/core_HeapAlloc.h>

// bspl includes
#include <mobius/bspl_KnotMultiset.h>

//-----------------------------------------------------------------------------

//! Tests conversion of vector to multiset and back.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_KnotMultiset::test_convert(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  const double U[] = {0, 0, 0, 1, 2, 3, 4, 4, 5, 5, 5};
  const int    nU  = sizeof(U)/sizeof(double);

  std::vector<double> U_vec;
  for ( size_t i = 0; i < nU; ++i )
    U_vec.push_back(U[i]);

  // Convert to multiset
  t_knot_multiset mset(U_vec);

  // Convert back to vector
  std::vector<double> U_back = mset.Convert();

  // Referential multiset
  t_knot_multiset ref_mset;
  ref_mset.AddKnot(0, 3);
  ref_mset.AddKnot(1, 1);
  ref_mset.AddKnot(2, 1);
  ref_mset.AddKnot(3, 1);
  ref_mset.AddKnot(4, 2);
  ref_mset.AddKnot(5, 3);

  // Compare multisets
  if ( ref_mset != mset )
    return res.failure();

  // Compare vectors
  if ( U_back != U_vec )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Tests union operation.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_KnotMultiset::test_unite(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  const double U[] = {0, 0, 0, 1, 2, 3, 4, 4, 5, 5, 5, 9};
  const double V[] = {0, 0, 1, 1, 1, 2, 2, 3, 4, 4, 5, 5, 5, 8};
  const int    nU  = sizeof(U)/sizeof(double);
  const int    nV  = sizeof(V)/sizeof(double);

  std::vector<double> U_vec;
  for ( size_t i = 0; i < nU; ++i )
    U_vec.push_back(U[i]);

  std::vector<double> V_vec;
  for ( size_t i = 0; i < nV; ++i )
    V_vec.push_back(V[i]);

  // Convert to multisets
  t_knot_multiset U_mset(U_vec);
  t_knot_multiset V_mset(V_vec);

  // Unite
  t_knot_multiset UV_mset = U_mset + V_mset;

  // Referential multiset
  t_knot_multiset ref_mset;
  ref_mset.AddKnot(0, 3);
  ref_mset.AddKnot(1, 3);
  ref_mset.AddKnot(2, 2);
  ref_mset.AddKnot(3, 1);
  ref_mset.AddKnot(4, 2);
  ref_mset.AddKnot(5, 3);
  ref_mset.AddKnot(8, 1);
  ref_mset.AddKnot(9, 1);

  // Compare multisets
  if ( ref_mset != UV_mset )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Tests subtraction.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_KnotMultiset::test_subtract(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  const double U[] = {0, 0, 0, 1, 2, 3, 4, 4, 5, 5, 5, 9, 10, 10};
  const double V[] = {0, 0, 1, 1, 1, 2, 2, 3, 4, 4, 5, 5, 5, 8};
  const int    nU  = sizeof(U)/sizeof(double);
  const int    nV  = sizeof(V)/sizeof(double);

  std::vector<double> U_vec;
  for ( size_t i = 0; i < nU; ++i )
    U_vec.push_back(U[i]);

  std::vector<double> V_vec;
  for ( size_t i = 0; i < nV; ++i )
    V_vec.push_back(V[i]);

  // Convert to multisets
  t_knot_multiset U_mset(U_vec);
  t_knot_multiset V_mset(V_vec);

  // Unite
  t_knot_multiset UV_mset = U_mset - V_mset;

  // Referential multiset
  t_knot_multiset ref_mset;
  ref_mset.AddKnot(0, 1);
  ref_mset.AddKnot(9, 1);
  ref_mset.AddKnot(10, 2);

  // Compare multisets
  if ( ref_mset != UV_mset )
    return res.failure();

  return res.success();
}

//-----------------------------------------------------------------------------

//! Tests complex case consisting in finding addendum to a given knot
//! vector, so that it becomes compatible with other ones.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
mobius::outcome
  mobius::test_KnotMultiset::test_find_addendum(const int funcID)
{
  outcome res( DescriptionFn(), funcID );

  const double U[] = {0, 0, 0, 1, 2, 3};
  const double V[] = {2, 2, 3, 5, 6};
  const double W[] = {0, 4, 8};
  const int    nU  = sizeof(U)/sizeof(double);
  const int    nV  = sizeof(V)/sizeof(double);
  const int    nW  = sizeof(W)/sizeof(double);

  std::vector<double> U_vec;
  for ( size_t i = 0; i < nU; ++i )
    U_vec.push_back(U[i]);

  std::vector<double> V_vec;
  for ( size_t i = 0; i < nV; ++i )
    V_vec.push_back(V[i]);

  std::vector<double> W_vec;
  for ( size_t i = 0; i < nW; ++i )
    W_vec.push_back(W[i]);

  // Convert to multisets
  t_knot_multiset U_mset(U_vec);
  t_knot_multiset V_mset(V_vec);
  t_knot_multiset W_mset(W_vec);

  // Unite
  t_knot_multiset UV_mset = V_mset + W_mset - U_mset;

  // Referential multiset
  t_knot_multiset ref_mset;
  ref_mset.AddKnot(2, 1);
  ref_mset.AddKnot(4, 1);
  ref_mset.AddKnot(5, 1);
  ref_mset.AddKnot(6, 1);
  ref_mset.AddKnot(8, 1);

  // Compare multisets
  if ( ref_mset != UV_mset )
    return res.failure();

  return res.success();
}
