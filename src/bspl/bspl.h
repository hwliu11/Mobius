//-----------------------------------------------------------------------------
// Created on: 19 July 2017
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

#ifndef bspl_HeaderFile
#define bspl_HeaderFile

// core includes
#include <mobius/core.h>

#define mobiusBSpl_NotUsed(x) x

#if defined _WIN32
  #if defined mobiusBSpl_EXPORTS
    #define mobiusBSpl_EXPORT __declspec(dllexport)
  #else
    #define mobiusBSpl_EXPORT __declspec(dllimport)
  #endif
#else
  #define mobiusBSpl_EXPORT
#endif

//-----------------------------------------------------------------------------
// DOXY group definition
//-----------------------------------------------------------------------------
//! \defgroup MOBIUS_BSPL B-spline library
//!
//! Data structures and algorithms for operating with B-spline functions.
//-----------------------------------------------------------------------------

namespace mobius {

//! \ingroup MOBIUS_BSPL
//!
//! Generic tools.
namespace bspl
{
  //! Returns last knot index (0-based).
  //! \param n [in] index of the last pole (0-based).
  //! \param p [in] degree of the B-spline basis function to use.
  //! \return last knot index.
  mobiusBSpl_EXPORT int
    M(const int n, const int p);

  //! Returns number of knots for the given indices related to the
  //! B-spline degree and number of control points to be used for
  //! a derived B-object. This is nothing more than a convenience method which
  //! you could prefer to use! instead of explicit well-known "m = n + p + 1"
  //! formulation.
  //! \param n [in] index of the last pole (0-based).
  //! \param p [in] degree of the B-spline basis function to use.
  //! \return number of knots.
  mobiusBSpl_EXPORT int
    NumberOfKnots(const int n, const int p);

  //! Checks whether the given number of poles in enough for a B-object
  //! with the passed degree.
  //! \param n [in] index of the last pole (0-based).
  //! \param p [in] degree of the B-spline basis function to use.
  //! \return true/false.
  mobiusBSpl_EXPORT bool
    Check(const int n, const int p);
};

};

#endif
