//-----------------------------------------------------------------------------
// Created on: 06 March 2015
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

// Own include
#include <QrTest_BasisKnotMultiset.h>

// QrCore includes
#include <QrCore_HeapAlloc.h>

// QrBSpl includes
#include <QrBSpl_BasisKnotMultiset.h>

//! Tests conversion of vector to multiset and back.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
bool QrTest_BasisKnotMultiset::test_convert(const int QrTest_NotUsed(funcID))
{
  const double U[] = {0, 0, 0, 1, 2, 3, 4, 4, 5, 5, 5};
  const int    nU  = sizeof(U)/sizeof(double);

  std::vector<double> U_vec;
  for ( size_t i = 0; i < nU; ++i )
    U_vec.push_back(U[i]);

  // Convert to multiset
  knot_multiset mset(U_vec);

  // Convert back to vector
  std::vector<double> U_back = mset.Convert();

  // Referential multiset
  knot_multiset ref_mset;
  ref_mset.AddKnot(0, 3);
  ref_mset.AddKnot(1, 1);
  ref_mset.AddKnot(2, 1);
  ref_mset.AddKnot(3, 1);
  ref_mset.AddKnot(4, 2);
  ref_mset.AddKnot(5, 3);

  // Compare multisets
  if ( ref_mset != mset )
    return false;

  // Compare vectors
  if ( U_back != U_vec )
    return false;

  return true;
}

//! Tests union operation.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
bool QrTest_BasisKnotMultiset::test_unite(const int QrTest_NotUsed(funcID))
{
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
  knot_multiset U_mset(U_vec);
  knot_multiset V_mset(V_vec);

  // Unite
  knot_multiset UV_mset = U_mset + V_mset;

  // Referential multiset
  knot_multiset ref_mset;
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
    return false;

  return true;
}

//! Tests subtraction.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
bool QrTest_BasisKnotMultiset::test_subtract(const int QrTest_NotUsed(funcID))
{
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
  knot_multiset U_mset(U_vec);
  knot_multiset V_mset(V_vec);

  // Unite
  knot_multiset UV_mset = U_mset - V_mset;

  // Referential multiset
  knot_multiset ref_mset;
  ref_mset.AddKnot(0, 1);
  ref_mset.AddKnot(9, 1);
  ref_mset.AddKnot(10, 2);

  // Compare multisets
  if ( ref_mset != UV_mset )
    return false;

  return true;
}

//! Tests complex case consisting in finding addendum to a given knot
//! vector, so that it becomes compatible with other ones.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
bool QrTest_BasisKnotMultiset::test_find_addendum(const int QrTest_NotUsed(funcID))
{
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
  knot_multiset U_mset(U_vec);
  knot_multiset V_mset(V_vec);
  knot_multiset W_mset(W_vec);

  // Unite
  knot_multiset UV_mset = V_mset + W_mset - U_mset;

  // Referential multiset
  knot_multiset ref_mset;
  ref_mset.AddKnot(2, 1);
  ref_mset.AddKnot(4, 1);
  ref_mset.AddKnot(5, 1);
  ref_mset.AddKnot(6, 1);
  ref_mset.AddKnot(8, 1);

  // Compare multisets
  if ( ref_mset != UV_mset )
    return false;

  return true;
}
