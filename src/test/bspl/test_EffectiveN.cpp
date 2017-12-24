//-----------------------------------------------------------------------------
// Created on: 01 August 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

// Own include
#include <mobius/test_EffectiveN.h>

// bspl includes
#include <mobius/bspl_EffectiveN.h>
#include <mobius/bspl_FindSpan.h>

//! Test scenario 001.
//! \param funcID [in] ID of the Test Function.
//! \return true in case of success, false -- otherwise.
bool mobius::test_EffectiveN::test1(const int funcID)
{
  const std::vector<double> U = {0.0, 0.0, 0.0, 1, 2, 3, 4, 4, 5, 5, 5};
  const int p = 2;

  bspl_EffectiveN Eval;

  double u = 2.5;
  bspl_FindSpan FindSpan(U, p);
  const int i = FindSpan(u);

  double* N = new double[p+1];
  Eval(u, U, p, i, N);

  // Set description variables
  SetVarDescr("U", U,      ID(), funcID);
  SetVarDescr("p", p,      ID(), funcID);
  SetVarDescr("u", u,      ID(), funcID);
  SetVarDescr("i", i,      ID(), funcID);
  SetVarDescr("N", N, p+1, ID(), funcID);

  if ( N[0] != 0.125 || N[1] != 0.75 || N[2] != 0.125 )
  {
    delete[] N;
    return false;
  }

  delete[] N;
  return true;
}
