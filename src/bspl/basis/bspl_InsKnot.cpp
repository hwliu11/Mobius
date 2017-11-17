//-----------------------------------------------------------------------------
// Created on: 05 March 2015
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

// Own include
#include <mobius/bspl_InsKnot.h>

// core includes
#include <mobius/core_HeapAlloc.h>

//! Implements knot insertion algorithm.
//! \param np [in]  index of the last pole in original curve.
//! \param p  [in]  degree.
//! \param UP [in]  original knot vector.
//! \param Pw [in]  original poles.
//! \param u  [in]  knot to insert.
//! \param k  [in]  knot interval.
//! \param s  [in]  original multiplicity.
//! \param r  [in]  how many times to insert.
//! \param nq [out] index of the last pole in resulting curve.
//! \param UQ [out] resulting knot vector.
//! \param Qw [out] resulting poles.
//! \return true in case of success, false -- otherwise.
bool mobius::bspl_InsKnot::operator()(const int                  np,
                                      const int                  p,
                                      const std::vector<double>& UP,
                                      const std::vector<xyz>&    Pw,
                                      const double               u,
                                      const int                  k,
                                      const int                  s,
                                      const int                  r,
                                      int&                       nq,
                                      std::vector<double>&       UQ,
                                      std::vector<xyz>&          Qw) const
{
  if ( r + s > p )
    return false; // Resulting multiplicity should not be greater than degree

  const int mp = np + p + 1;
  nq = np + r;

  // Internal array
  std::vector<xyz> Rw(p + 1);
  for ( int i = 0; i <= nq; ++i )
    Qw.push_back( xyz() );

  // Load new knot vector
  for ( int i = 0;     i <= k;  ++i ) UQ[i]     = UP[i]; // Reused
  for ( int i = 1;     i <= r;  ++i ) UQ[k + i] = u;     // Inserted
  for ( int i = k + 1; i <= mp; ++i ) UQ[i + r] = UP[i]; // Reused

  // Save unaltered control points
  for ( int i = 0;     i <= k - p; ++i ) Qw[i]     = Pw[i]; // Reused
  for ( int i = k - s; i <= np;    ++i ) Qw[i + r] = Pw[i]; // Reused
  for ( int i = 0;     i <= p - s; ++i ) Rw[i]     = Pw[k - p + i];

  // Insert the knot r times
  int L = 0;
  for ( int j = 1; j <= r; ++j )
  {
    L = k - p + j;
    for ( int i = 0; i <= p - j - s; ++i )
    {
      const double alpha = (u - UP[L + i]) / (UP[i + k +1] - UP[L + i]);
      Rw[i] = Rw[i + 1]*alpha + Rw[i]*(1.0 - alpha);
    }
    Qw[L]             = Rw[0];
    Qw[k + r - j - s] = Rw[p - j - s];
  }

  // Load remaining control points
  for ( int i = L + 1; i < k - s; ++i )
    Qw[i] = Rw[i - L];

  return true;
}
