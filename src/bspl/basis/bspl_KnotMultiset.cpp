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
#include <mobius/bspl_KnotMultiset.h>

// STL includes
#include <algorithm>

//! Empty constructor.
mobius::bspl_KnotMultiset::bspl_KnotMultiset()
{}

//! Assignment constructor.
//! \param U_mset [in] vector to copy values from.
mobius::bspl_KnotMultiset::bspl_KnotMultiset(const bspl_KnotMultiset& U_mset)
{
  this->operator=(U_mset);
}

//! Constructor. Creates multiset under the given raw knot vector.
//! \param U [in] original knot vector with multiplicities as repetitions of
//!               the corresponding values.
mobius::bspl_KnotMultiset::bspl_KnotMultiset(const std::vector<double>& U)
{
  if ( !U.size() )
    return;

  // Populate multiset
  int step = 0;
  const size_t m = U.size() - 1;
  for ( size_t i = 0; i <= m; i += step )
  {
    elem el(U[i], 1);
    for ( size_t j = i + 1; j <= m; ++j )
      if ( U[j] == U[i] )
        el.m++;

    step = el.m;
    m_mset.push_back(el);
  }
}

//! Compares two multisets.
//! \param U_mset [in] multiset to compare with.
//! \return true in case of equality, false -- otherwise.
bool mobius::bspl_KnotMultiset::operator==(const bspl_KnotMultiset& U_mset) const
{
  if ( this->Size() != U_mset.Size() )
    return false;

  const size_t size = this->Size();
  for ( size_t i = 0; i < size; ++i )
  {
    if ( m_mset[i].u != U_mset.m_mset[i].u || m_mset[i].m != U_mset.m_mset[i].m )
      return false;
  }

  return true;
}

//! Compares two multisets.
//! \param U_mset [in] multiset to compare with.
//! \return true in case of inequality, false -- otherwise.
bool mobius::bspl_KnotMultiset::operator!=(const bspl_KnotMultiset& U_mset) const
{
  return !this->operator==(U_mset);
}

//! Assignment operator.
//! \param U_mset [in] vector to copy values from.
//! \return modified this.
mobius::bspl_KnotMultiset&
  mobius::bspl_KnotMultiset::operator=(const bspl_KnotMultiset& U_mset)
{
  this->m_mset = U_mset.m_mset;
  return *this;
}

//! Unites the passed multiset with this one.
//! \param V [in] second operand.
//! \return operation result.
mobius::bspl_KnotMultiset
  mobius::bspl_KnotMultiset::Unite(const bspl_KnotMultiset& V) const
{
  // Populate the result
  bspl_KnotMultiset result = *this;
  for ( size_t i = 0; i < V.m_mset.size(); ++i )
  {
    elem elem_in_result, elem_in_V = V.m_mset[i];
    const int idx = result.find(elem_in_V.u, elem_in_result);

    if ( idx == -1 )
      result.m_mset.push_back(elem_in_V);
    else
      result.m_mset[idx].m = std::max(elem_in_result.m, elem_in_V.m);
  }

  // Sort the result
  std::sort(result.m_mset.begin(), result.m_mset.end(), __less);
  return result;
}

//! Unites the passed multiset with this one.
//! \param V [in] second operand.
//! \return operation result.
mobius::bspl_KnotMultiset
  mobius::bspl_KnotMultiset::operator+(const bspl_KnotMultiset& V) const
{
  return this->Unite(V);
}

//! Subtracts the passed multiset from this one.
//! \param V [in] second operand.
//! \return operation result.
mobius::bspl_KnotMultiset
  mobius::bspl_KnotMultiset::Subtract(const bspl_KnotMultiset& V) const
{
  // Populate the result
  bspl_KnotMultiset result = *this;
  for ( size_t i = 0; i < V.m_mset.size(); ++i )
  {
    elem elem_in_result, elem_in_V = V.m_mset[i];
    const int idx = result.find(elem_in_V.u, elem_in_result);

    if ( idx != -1 )
    {
      if ( elem_in_result.m < elem_in_V.m )
        result.m_mset.erase( result.m_mset.begin() + (size_t) idx );
      else
      {
        result.m_mset[idx].m -= elem_in_V.m;

        // Knot vanishes
        if ( !result.m_mset[idx].m )
          result.m_mset.erase( result.m_mset.begin() + idx );
      }
    }
  }
  return result;
}

//! Subtracts the passed multiset from this one.
//! \param V [in] second operand.
//! \return operation result.
mobius::bspl_KnotMultiset
  mobius::bspl_KnotMultiset::operator-(const bspl_KnotMultiset& V) const
{
  return Subtract(V);
}

//! Injects the passed multiset to this one. This is another kind of merging
//! which allows to sum up multisets just like ordinary sets. Formally,
//! the only difference between injection and summation is that here we do not
//! take the max multiplicity but simply add one multiplicity to another.
//! \param V [in] second operand.
//! \return operation result.
mobius::bspl_KnotMultiset
  mobius::bspl_KnotMultiset::Inject(const bspl_KnotMultiset& V) const
{
  // Populate the result
  bspl_KnotMultiset result = *this;
  for ( size_t i = 0; i < V.m_mset.size(); ++i )
  {
    elem elem_in_result, elem_in_V = V.m_mset[i];
    const int idx = result.find(elem_in_V.u, elem_in_result);

    if ( idx == -1 )
      result.m_mset.push_back(elem_in_V);
    else
      result.m_mset[idx].m = elem_in_result.m + elem_in_V.m;
  }

  // Sort the result
  std::sort(result.m_mset.begin(), result.m_mset.end(), __less);
  return result;
}

//! Injects the passed multiset into this one.
//! \param V [in] second operand.
//! \return operation result.
mobius::bspl_KnotMultiset
  mobius::bspl_KnotMultiset::operator^(const bspl_KnotMultiset& V) const
{
  return Inject(V);
}

//! Converts multiset to ordinary vector.
//! \return ordinary vector.
std::vector<double> mobius::bspl_KnotMultiset::Convert() const
{
  std::vector<double> result;
  for ( size_t i = 0; i < m_mset.size(); ++i )
  {
    for ( int j = 0; j < m_mset[i].m; ++j )
      result.push_back(m_mset[i].u);
  }
  return result;
}

//! Attempts to find a knot with the given value and any multiplicity in the
//! store multiset.
//! \param u     [in] value to search for.
//! \param found [in] found element (if any).
//! \return index of the element or -1 if nothing was found.
int mobius::bspl_KnotMultiset::find(const double u,
                                    elem&        found) const
{
  int res_idx = -1;
  for ( size_t i = 0; i < m_mset.size(); ++i )
  {
    if ( m_mset[i].u == u )
    {
      res_idx = (int) i;
      found = m_mset[i];
      break;
    }
  }

  return res_idx;
}
