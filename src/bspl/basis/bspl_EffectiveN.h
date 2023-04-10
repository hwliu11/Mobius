//-----------------------------------------------------------------------------
// Created on: 29 July 2013
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

#ifndef bspl_EffectiveN_HeaderFile
#define bspl_EffectiveN_HeaderFile

// bspl includes
#include <mobius/bspl.h>

namespace mobius {

//! \ingroup MOBIUS_BSPL
//!
//! \brief A2.2 from "The NURBS Book".
//!
//! Algorithm calculating those B-spline basis functions which are
//! non-vanishing in the given span. The target span is referenced by
//! its zero-based index. In order to use this functionality properly,
//! one should locate the target span by BasisFindSpan() and then ask this
//! routine to evaluate the basis functions for the passed parameter u.
//! This function performs effective evaluation, so as only those basis
//! functions which are not constant nulls are taken into account.
//! Moreover, this function reuses those values which are calculated at
//! previous stages of its internal iterations (the evaluation scheme
//! is principally sequential).
//!
//! To remind, the general formulation is as follows:
//!
//! <pre>
//!         | 1, u belongs to [u_i, u_{i+1}),
//! N_i,0 = |
//!         | 0, otherwise
//!
//!            u - u_i                     u_{i+p+1} - u
//! N_i,p = --------------- N_i,{p-1} + --------------------- N_{i+1},{p-1}
//!          u_{i+p} - u_i               u_{i+p+1} - u_{i+1}
//! </pre>
//!
//! Having p=1 in mind it is easy to understand this formula as simple
//! blending of a couple of 0-degree basis functions N_i,0 and N_i+1,0
//! defined on two adjacent intervals. While the basis 0-degree function
//! was not-null on a single [u_i, u_i+1) interval only, its successive
//! 1-degree function is non-vanishing on a double interval, i.e:
//!
//! <pre>
//!           u - u_i                   u_{i+2} - u
//! N_i,1 = -------------- N_i,0 +  ------------------- N_{i+1},0
//!         u_{i+1} - u_i            u_{i+2} - u_{i+1}
//! </pre>
//!
//! From this we can see that N_i,1 is nothing more than a linear combination
//! of lower-degree functions. In order to calculate the basis function of
//! order 1, we need to calculate two basis functions of order 0:
//!
//! <pre>
//!   N_i,0
//!          |
//!          N_i,1
//!          |
//!   N_{i+1},0
//! </pre>
//!
//! Extending these conclusions on the entire knot vector, we have the
//! following evaluation scheme:
//!
//! <pre>
//!   N_0,0
//!          |
//!          N_0,1
//!          |
//!   N_1,0
//!   ...
//!   N_i,0
//!          |
//!          N_i,1
//!          |
//! N_{i+1},0
//!   ...
//! N_{m-2},0
//!          |
//!          N_{m-2},1
//!          |
//! N_{m-1},0
//! </pre>
//!
//! Even from this triangular flow it is easy to see that in order to
//! preserve basisness property (all functions must give 1 in sum -- this
//! is not enough but necessary condition) we should repeat trailing and
//! leading knots (p + 1) times. Otherwise we will miss basis functions
//! for the starting and finishing intervals.
//!
//! From the same triangular scheme we can see that N_i,p function is
//! non-vanishing only in interval [u_i, u_{i+p+1}).
//!
//! The implemented approach is based on algorithm A2.2 from "The NURBS Book".
//! The main idea is to evaluate only those basis functions which give
//! any effect on the selected span. Such functions are all deduced from
//! the 0-degree basis function:
//!
//! <pre>
//!         N_{i-p},p
//!          /
//!        ...
//!        /
//!     N_{i-1},1
//!      /
//! N_i,0 --- span #i yields functions from (i-p) to p
//!      |
//!     N_i,1
//!        |
//!        ...
//!          |
//!         N_i,p
//! </pre>
//!
//! Indeed, for span {i} the only zero-degree non-vanishing basis function
//! is {N_i,0}. Now we want to know what are the non-vanishing functions
//! of degree {p}. From Cox-deBoor formulation we know that higher-degree
//! functions are expressed with lower-degree functions. The only functions
//! which depend on the non-vanishing {N_i,0} are illustrated by the
//! reversed triangular scheme given above. All other functions are depend
//! on null lower-degree ones, so we exclude them from consideration.
//!
//! According to the notes given in "The NURBS Book", it is possible
//! to re-use some terms during the evaluation procedure. The following
//! working variables are used:
//!
//! <pre>
//!   left[j]  = u - u_{i+1-j}
//!   right[j] = u_{i+j} - u
//! </pre>
//!
//! Here i is the 0-based index of the working span.
//!
//! \param u [in]  parameter to evaluate B-spline function for.
//! \param U [in]  knot vector.
//! \param p [in]  degree.
//! \param i [in]  0-based effective span index.
//! \param N [out] result.
//!
//! \todo complete description
//! \todo provide benchmarking between straightforward and optimized functions
//! \todo provide theoretical background for this optimization
class bspl_EffectiveN
{
public:

  mobiusBSpl_EXPORT void
    operator()(const double               u,
               const std::vector<double>& U,
               const int                  p,
               const int                  i,
               double*                    N) const;

};

}

#endif
