//-----------------------------------------------------------------------------
// Created on: 10 December 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2018-present, Sergey Slyadnev
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
#include <mobius/bspl_Decompose.h>

// core includes
#include <mobius/core_HeapAlloc.h>

//-----------------------------------------------------------------------------

bool mobius::bspl_Decompose::operator()(const int                        n,
                                        const int                        p,
                                        const std::vector<double>&       U,
                                        const std::vector<xyz>&          Pw,
                                        int&                             nb,
                                        std::vector< std::vector<xyz> >& Qw,
                                        std::vector<int>&                breakpoints) const
{
  const int m = n + p + 1;
  int       a = p;
  int       b = p + 1;

  // Dynamically allocate an array for alphas.
  ptr<alloc1d> localAlloc = new alloc1d;
  //
  double* alphas = localAlloc->Allocate(p, true);

  // Prepare array for control points.
  nb = 0;
  //
  Qw.push_back( std::vector<xyz>() );
  //
  for ( int i = 0; i <= p; ++i )
    Qw[nb].push_back( Pw[i] );

  // Main loop.
  while ( b < m )
  {
    int i = b;

    while ( b < m && U[b + 1] == U[b] ) b++;

    int mult = b - i + 1;

    if ( mult < p )
    {
      const double numer = U[b] - U[a]; // Numerator of alpha.

      // Compute and store alphas.
      for ( int j = p; j > mult; --j )
        alphas[j - mult - 1] = numer / (U[a + j] - U[a]);

      // Insert knot `r` times.
      const int r = p - mult;
      //
      for ( int j = 1; j <= r; ++j )
      {
        const int save = r - j;
        const int s    = mult + j; // This many new points.

        for ( int k = p; k >= s; --k )
        {
          const double alpha = alphas[k - s];
          Qw[nb][k] = alpha*Qw[nb][k] + (1.0 - alpha)*Qw[nb][k - 1];
        }

        if ( b < m )
        {
          // Enlarge output collection.
          if ( Qw.size() <= nb + 1 )
          {
            Qw.push_back( std::vector<xyz>() );
            Qw[nb + 1].resize(p + 1);
          }

          // Control point of the next segment.
          Qw[nb + 1][save] = Qw[nb][p];
        }
      }
    }

    nb = nb + 1; // Bezier segment completed.

    // Add breakpoint index.
    breakpoints.push_back(i);

    if ( b < m )
    {
      // Initialize for the next segment.
      for ( int ii = p - mult; ii <= p; ++ii )
        Qw[nb][ii] = Pw[b - p + ii];

      a = b;
      b = b + 1;
    }
  }

  return true;
}
