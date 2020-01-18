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

#ifndef bspl_KnotMultiset_HeaderFile
#define bspl_KnotMultiset_HeaderFile

// bspl includes
#include <mobius/bspl.h>

// core includes
#include <mobius/core_XYZ.h>

namespace mobius {

//! \ingroup MOBIUS_BSPL
//!
//! Knot vector as a multiset (entry + multiplicity).
class bspl_KnotMultiset
{
public:

  //! Handy structure to associate knot value with its multiplicity.
  struct elem
  {
    double u; //!< Parameter value.
    int    m; //!< Multiplicity.

    //! Default constructor.
    elem() : u(0.0), m(0) {}

    //! Complete constructor.
    //! \param _knot [in] knot value.
    //! \param _mult [in] multiplicity.
    elem(const double _knot,
         const int    _mult) : u(_knot), m(_mult) {}
  };

// Construction & destruction:
public:

  mobiusBSpl_EXPORT
    bspl_KnotMultiset();

  mobiusBSpl_EXPORT
    bspl_KnotMultiset(const bspl_KnotMultiset& U_mset);

  mobiusBSpl_EXPORT
    bspl_KnotMultiset(const std::vector<double>& U);

public:

  mobiusBSpl_EXPORT bool
    operator==(const bspl_KnotMultiset& U_mset) const;

  mobiusBSpl_EXPORT bool
    operator!=(const bspl_KnotMultiset& U_mset) const;

  mobiusBSpl_EXPORT bspl_KnotMultiset&
    operator=(const bspl_KnotMultiset& U_mset);

  mobiusBSpl_EXPORT bspl_KnotMultiset
    Unite(const bspl_KnotMultiset& V) const;

  mobiusBSpl_EXPORT bspl_KnotMultiset
    operator+(const bspl_KnotMultiset& V) const;

  mobiusBSpl_EXPORT bspl_KnotMultiset
    Subtract(const bspl_KnotMultiset& V) const;

  mobiusBSpl_EXPORT bspl_KnotMultiset
    operator-(const bspl_KnotMultiset& V) const;

  mobiusBSpl_EXPORT bspl_KnotMultiset
    Inject(const bspl_KnotMultiset& V) const;

  mobiusBSpl_EXPORT bspl_KnotMultiset
    operator^(const bspl_KnotMultiset& V) const;

  mobiusBSpl_EXPORT std::vector<double>
    Convert() const;

public:

  //! \return size of multiset.
  size_t Size() const { return m_mset.size(); }

  //! Returns knot by its 0-based index.
  //! \param i [in] 0-based index of knot.
  //! \return knot with multiplicity.
  const elem& Knot(const size_t i) const { return m_mset[i]; }

  //! Returns knot by its 0-based index. The knot can be modified then.
  //! \param i [in] 0-based index of knot.
  //! \return reference to the knot with multiplicity.
  elem& ChangeKnot(const size_t i) { return m_mset[i]; }

  //! Adds new knot to the end of multiset. No sorting is done.
  //! \param u [in] knot value.
  //! \param m [in] multiplicity.
  void AddKnot(const double u, const int m) { m_mset.push_back( elem(u, m) ); }

private:

  int find(const double u,
           elem&        found) const;

private:

  //! Compares two knots by value.
  //! \param e1 [in] first knot.
  //! \param e2 [in] second knot.
  //! \return true if e1 < e2.
  static bool __less(const elem& e1, const elem& e2) { return e1.u < e2.u; }

protected:

  std::vector<elem> m_mset; //!< Multiset.

};

//! \ingroup MOBIUS_BSPL
//!
//! Handy shortcuts.
typedef bspl_KnotMultiset t_knot_multiset;

}

#endif
