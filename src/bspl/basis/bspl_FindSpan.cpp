//-----------------------------------------------------------------------------
// Created on: 11 June 2013
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

// bspl includes
#include <mobius/bspl_FindSpan.h>

#undef COUT_DEBUG
#if defined COUT_DEBUG
  #pragma message("===== warning: COUT_DEBUG is enabled")
#endif

//-----------------------------------------------------------------------------

//! Initializes the tool with all necessary B-spline properties.
//! \param[in] U knot vector.
//! \param[in] p degree of the corresponding basis splines.
mobius::bspl_FindSpan::bspl_FindSpan(const std::vector<double>& U,
                                     const int                  p)
: m_U(U), m_iDeg(p)
{}

//-----------------------------------------------------------------------------

//! Finds the target span index by binary search.
//! \param[in] u target parameter.
//! \return span index.
int mobius::bspl_FindSpan::operator()(const double u) const
{
  int firstNonZeroIndex = 0; // Unused.
  return this->operator()(u, firstNonZeroIndex);
}

//-----------------------------------------------------------------------------

//! Finds the target span index by binary search.
//! \param[in]  u                 parameter in question.
//! \param[out] firstNonZeroIndex index of the first non-zero basis spline.
//! \return span index.
int mobius::bspl_FindSpan::operator()(const double u,
                                      int&         firstNonZeroIndex) const
{
  const int nU = (int) m_U.size();
  const int p  = m_iDeg;
  const int m  = nU - 1;
  const int n  = m - p - 1;
  //
  if ( u <= m_U[0] )
  {
    firstNonZeroIndex = 0;
    return p;
  }
  //
  if ( u >= m_U[n + 1] )
  {
    firstNonZeroIndex = n - p;
    return n;
  }

  int  mid_idx;
  int  min_idx = 0;
  int  max_idx = nU;
  bool isFound = false;

#if defined COUT_DEBUG
  std::cout << "\tu = " << u << std::endl;
#endif

  do
  {
    mid_idx = (min_idx + max_idx) / 2;

    if ( mid_idx == min_idx || mid_idx == max_idx )
    {
      isFound = true;
      break;
    }

    const double mid_u = m_U[mid_idx];

#if defined COUT_DEBUG
    std::cout << "\tmid_u = " << mid_u << std::endl;
#endif

    if ( mid_u <= u )
      min_idx = mid_idx;
    else if ( mid_u > u )
      max_idx = mid_idx;
  }
  while ( !isFound );

  firstNonZeroIndex = mid_idx;
  return mid_idx;
}
