//-----------------------------------------------------------------------------
// Created on: 05 March 2015
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
#include <mobius/bspl_InsKnot.h>

// core includes
#include <mobius/core_HeapAlloc.h>

//-----------------------------------------------------------------------------

bool mobius::bspl_InsKnot::operator()(const int                  np,
                                      const int                  p,
                                      const std::vector<double>& UP,
                                      const std::vector<t_xyz>&  Pw,
                                      const double               u,
                                      const int                  k,
                                      const int                  s,
                                      const int                  r,
                                      int&                       nq,
                                      std::vector<double>&       UQ,
                                      std::vector<t_xyz>&        Qw) const
{
  if ( r + s > p )
    return false; // Resulting multiplicity should not be greater than degree.

  const int mp = np + p + 1;
  nq = np + r;

  // Internal array.
  std::vector<t_xyz> Rw(p + 1);
  UQ.resize(mp + r + 1);
  Qw.resize(nq + 1);

  // Load new knot vector.
  for ( int i = 0;     i <= k;  ++i ) UQ[i]     = UP[i]; // Reused.
  for ( int i = 1;     i <= r;  ++i ) UQ[k + i] = u;     // Inserted.
  for ( int i = k + 1; i <= mp; ++i ) UQ[i + r] = UP[i]; // Reused.

  // Save unaltered control points.
  for ( int i = 0;     i <= k - p; ++i ) Qw[i]     = Pw[i]; // Reused.
  for ( int i = k - s; i <= np;    ++i ) Qw[i + r] = Pw[i]; // Reused.
  for ( int i = 0;     i <= p - s; ++i ) Rw[i]     = Pw[k - p + i]; // Add auxiliary control points.

  // Insert the knot r times.
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

  // Load remaining control points.
  for ( int i = L + 1; i < k - s; ++i )
    Qw[i] = Rw[i - L];

  return true;
}

//-----------------------------------------------------------------------------

bool mobius::bspl_InsKnot::operator()(const int                                np,
                                      const int                                p,
                                      const std::vector<double>&               UP,
                                      const int                                mp,
                                      const int                                q,
                                      const std::vector<double>&               VP,
                                      const std::vector< std::vector<t_xyz> >& Pw,
                                      const bspl_ParamDirection                dir,
                                      const double                             knot,
                                      const int                                k,
                                      const int                                s,
                                      const int                                r,
                                      int&                                     nq,
                                      std::vector<double>&                     UQ,
                                      int&                                     mq,
                                      std::vector<double>&                     VQ,
                                      std::vector< std::vector<t_xyz> >&       Qw) const
{
  if ( dir == ParamDirection_Undefined )
    return false; // Contract check.

  t_ptr<t_alloc2d> localAlloc = new t_alloc2d;

  // Process insertion in U direction.
  if ( dir == ParamDirection_U )
  {
    // Allocate array for alphas.
    double** alpha = localAlloc->Allocate(p - s, r + 1, true);

    // Number of rows and columns.
    nq = np + r; // New index of last pole in U direction.
    mq = mp;     // Index of last pole in U direction.

    // Internal array.
    std::vector<t_xyz> Rw(p + 1);
    Qw.resize(nq + 1);
    //
    for ( size_t i = 0; i < Qw.size(); ++i )
    {
      std::vector<t_xyz> pts; pts.resize(mq + 1);
      Qw[i] = pts;
    }

    // Prepare UQ (new knot vector).
    const int midx = np + p + 1;
    UQ.resize(midx + r + 1); // UQ is greater than the original UP.

    // Copy v-vector into VQ.
    VQ = VP;

    // Load u-vector as in A5.1: new knot is inserted here.
    for ( int i = 0;     i <= k;    ++i ) UQ[i]     = UP[i]; // Reused.
    for ( int i = 1;     i <= r;    ++i ) UQ[k + i] = knot;  // Inserted.
    for ( int i = k + 1; i <= midx; ++i ) UQ[i + r] = UP[i]; // Reused.

    // Save the alphas.
    for ( int j = 1; j <= r; ++j )
    {
      const int L = k - p + j;
      for ( int i = 0; i <= (p - j - s); ++i )
        alpha[i][j] = (knot - UP[L + i]) / (UP[i + k + 1] - UP[L + i]);
    }

    // For each row do.
    for ( int row = 0; row <= mp; ++row )
    {
      // Save unaltered control points.
      for ( int i = 0;     i <= k - p; ++i ) Qw[i][row]     = Pw[i][row]; // Reused.
      for ( int i = k - s; i <= np;    ++i ) Qw[i + r][row] = Pw[i][row]; // Reused.
      for ( int i = 0;     i <= p - s; ++i ) Rw[i]          = Pw[k - p + i][row]; // Add auxiliary control points.

      // Insert the knot r times.
      int L = 0;
      for ( int j = 1; j <= r; ++j )
      {
        L = k - p + j;
        for ( int i = 0; i <= p - j - s; ++i )
        {
          Rw[i] = alpha[i][j]*Rw[i + 1] + (1. - alpha[i][j])*Rw[i];
        }
        Qw[L][row]             = Rw[0];
        Qw[k + r - j - s][row] = Rw[p - j - s];
      }

      // Load the resulting control points.
      for ( int i = L + 1; i < k - s; ++i )
        Qw[i][row] = Rw[i - L];
    }
  }

  // Process insertion in V direction.
  else if ( dir == ParamDirection_V )
  {
    // Allocate array for alphas.
    double** alpha = localAlloc->Allocate(q - s, r + 1, true);

    // Number of rows and columns.
    nq = np;     // Index of last pole in U direction.
    mq = mp + r; // New index of last pole in V direction.

    // Internal array.
    std::vector<t_xyz> Rw(q + 1);
    Qw.resize(nq + 1); // Number of rows does not change in V insertion (it's the same as in Pw).
    //
    for ( size_t i = 0; i < Qw.size(); ++i )
    {
      std::vector<t_xyz> pts; pts.resize(mq + 1); // The number of poles in columns increases on `r`.
      Qw[i] = pts;
    }

    // Prepare VQ (new knot vector).
    const int midx = mp + q + 1;
    VQ.resize(midx + r + 1); // VQ is greater than the original VP.

    // Copy u-vector into UQ.
    UQ = UP;

    // Load v-vector as in A5.1: new knot is inserted here.
    for ( int i = 0;     i <= k;    ++i ) VQ[i]     = VP[i]; // Reused.
    for ( int i = 1;     i <= r;    ++i ) VQ[k + i] = knot;  // Inserted.
    for ( int i = k + 1; i <= midx; ++i ) VQ[i + r] = VP[i]; // Reused.

    // Save the alphas.
    for ( int j = 1; j <= r; ++j )
    {
      const int L = k - q + j;
      for ( int i = 0; i <= (q - j - s); ++i )
        alpha[i][j] = (knot - VP[L + i]) / (VP[i + k + 1] - VP[L + i]);
    }

    // For each col do.
    for ( int col = 0; col <= np; ++col )
    {
      // Save unaltered control points.
      for ( int i = 0;     i <= k - q; ++i ) Qw[col][i]     = Pw[col][i]; // Reused.
      for ( int i = k - s; i <= mp;    ++i ) Qw[col][i + r] = Pw[col][i]; // Reused.
      for ( int i = 0;     i <= q - s; ++i ) Rw[i]          = Pw[col][k - q + i]; // Add auxiliary control points.

      // Insert the knot r times.
      int L = 0;
      for ( int j = 1; j <= r; ++j )
      {
        L = k - q + j;
        for ( int i = 0; i <= q - j - s; ++i )
        {
          Rw[i] = alpha[i][j]*Rw[i + 1] + (1. - alpha[i][j])*Rw[i];
        }
        Qw[col][L]             = Rw[0];
        Qw[col][k + r - j - s] = Rw[q - j - s];
      }

      // Load the resulting control points.
      for ( int i = L + 1; i < k - s; ++i )
        Qw[col][i] = Rw[i - L];
    }
  }

  return true;
}
