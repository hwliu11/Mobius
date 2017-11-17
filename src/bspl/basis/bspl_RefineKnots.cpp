//-----------------------------------------------------------------------------
// Created on: 06 March 2015
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

// Own include
#include <mobius/bspl_RefineKnots.h>

// core includes
#include <mobius/core_HeapAlloc.h>

// bspl includes
#include <mobius/bspl_FindSpan.h>

//! Implements knot insertion algorithm.
//! \param n    [in]  index of the last pole in original curve.
//! \param p    [in]  degree.
//! \param U    [in]  original knot vector.
//! \param Pw   [in]  original poles.
//! \param X    [in]  knots to insert.
//! \param r    [in]  index of the last element in X.
//! \param Ubar [out] resulting knot vector.
//! \param Qw   [out] resulting poles.
//! \return true in case of success, false -- otherwise.
bool mobius::bspl_RefineKnots::operator()(const int                  n,
                                          const int                  p,
                                          const std::vector<double>& U,
                                          const std::vector<xyz>&    Pw,
                                          const double*              X,
                                          const int                  r,
                                          std::vector<double>&       Ubar,
                                          std::vector<xyz>&          Qw) const
{
  const int m  = n + p + 1;
  const int nq = n + r + 1;

  bspl_FindSpan FindSpan(U, p);
  int a = FindSpan(X[0]);
  int b = FindSpan(X[r]);
  b = b + 1;

  for ( int i = 0; i <= nq; ++i )
    Qw.push_back( xyz() );

  for ( int j = 0;     j <= a - p; ++j ) Qw[j]           = Pw[j];
  for ( int j = b - 1; j <= n;     ++j ) Qw[j + r + 1]   = Pw[j];
  for ( int j = 0;     j <= a;     ++j ) Ubar[j]         = U[j];
  for ( int j = b + p; j <= m;     ++j ) Ubar[j + r + 1] = U[j];

  int i = b + p - 1;
  int k = b + p + r;

  for ( int j = r; j >= 0; --j )
  {
    while ( X[j] <= U[i] && i > a )
    {
      Qw[k - p - 1] = Pw[i - p - 1];
      Ubar[k]       = U[i];
      k             = k - 1;
      i             = i - 1;
    }
    Qw[k - p - 1] = Qw[k - p];
    for ( int l = 1; l <= p; ++l )
    {
      const int ind   = k - p + l;
      double    alpha = Ubar[k + l] - X[j];

      if ( fabs(alpha) == 0.0 )
        Qw[ind - 1] = Qw[ind];
      else
      {
        alpha       = alpha/(Ubar[k + l] - U[i - p + l]);
        Qw[ind - 1] = Qw[ind - 1]*alpha + Qw[ind]*(1.0 - alpha);
      }
    }
    Ubar[k] = X[j];
    k       = k - 1;
  }

  return true;
}
