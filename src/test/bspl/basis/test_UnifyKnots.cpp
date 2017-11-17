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
#include <QrTest_BasisUnifyKnots.h>

// QrCore includes
#include <QrCore_HeapAlloc.h>

// QrBSpl includes
#include <QrBSpl_BasisKnotMultiset.h>
#include <QrBSpl_BasisUnifyKnots.h>

//! Tests unification.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
bool QrTest_BasisUnifyKnots::test_unify(const int QrTest_NotUsed(funcID))
{
  const double U[] = {0, 0, 1, 2, 3};
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

  std::vector< std::vector<double> > knot_vectors;
  knot_vectors.push_back(U_vec);
  knot_vectors.push_back(V_vec);
  knot_vectors.push_back(W_vec);

  QrBSpl_BasisUnifyKnots<double> Unify;
  std::vector< std::vector<double> > X = Unify(knot_vectors);

  // Referential multiset
  std::vector< std::vector<double> > refs;
  std::vector<double> ref0;
  ref0.push_back(2);
  ref0.push_back(4);
  ref0.push_back(5);
  ref0.push_back(6);
  ref0.push_back(8);
  refs.push_back(ref0);
  std::vector<double> ref1;
  ref1.push_back(0);
  ref1.push_back(0);
  ref1.push_back(1);
  ref1.push_back(4);
  ref1.push_back(8);
  refs.push_back(ref1);
  std::vector<double> ref2;
  ref2.push_back(0);
  ref2.push_back(1);
  ref2.push_back(2);
  ref2.push_back(2);
  ref2.push_back(3);
  ref2.push_back(5);
  ref2.push_back(6);
  refs.push_back(ref2);

  //----------------------
  // Compare knot vectors
  //----------------------

  if ( X.size() != refs.size() )
    return false;

  for ( size_t i = 0; i < refs.size(); ++i )
  {
    if ( refs[i] != X[i] )
      return false;
  }

  //----------------------------------------------
  // Apply addendums to have unified knot vectors
  //----------------------------------------------

  // Original and eXtension vectors
  knot_multiset U_o(U_vec), V_o(V_vec), W_o(W_vec), U_x(X[0]), V_x(X[1]), W_x(X[2]);

  // Apply extensions
  std::vector<double> U_final = (U_o ^ U_x).Convert();
  std::vector<double> V_final = (V_o ^ V_x).Convert();
  std::vector<double> W_final = (W_o ^ W_x).Convert();

  // Now we should obtain just the same knot vectors. Let's check it
  std::vector<double> ref_unified;
  ref_unified.push_back(0);
  ref_unified.push_back(0);
  ref_unified.push_back(1);
  ref_unified.push_back(2);
  ref_unified.push_back(2);
  ref_unified.push_back(3);
  ref_unified.push_back(4);
  ref_unified.push_back(5);
  ref_unified.push_back(6);
  ref_unified.push_back(8);

  // Verify
  if ( U_final != ref_unified || V_final != ref_unified || W_final != ref_unified )
    return false;

  return true;
}
