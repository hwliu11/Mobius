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

#ifndef bspl_InsKnot_HeaderFile
#define bspl_InsKnot_HeaderFile

// bspl includes
#include <mobius/bspl_ParamDirection.h>

// core includes
#include <mobius/core_UV.h>
#include <mobius/core_XYZ.h>

namespace mobius {

//! \ingroup MOBIUS_BSPL
//!
//! Knot insertion implementation according to A5.1 (curves) and
//! A5.3 (surfaces) algorithms from "The NURBS Book".
class bspl_InsKnot
{
public:

  //! Implements knot insertion algorithm for curves (A5.1).
  //!
  //! \param[in]  np index of the last pole in the original curve.
  //! \param[in]  p  degree.
  //! \param[in]  UP original knot vector.
  //! \param[in]  Pw original poles.
  //! \param[in]  u  knot to insert.
  //! \param[in]  k  knot interval.
  //! \param[in]  s  original multiplicity.
  //! \param[in]  r  how many times to insert.
  //! \param[out] nq index of the last pole in the resulting curve.
  //! \param[out] UQ resulting knot vector.
  //! \param[out] Qw resulting poles.
  //!
  //! \return true in case of success, false -- otherwise.
  mobiusBSpl_EXPORT bool
    operator()(const int                  np,
               const int                  p,
               const std::vector<double>& UP,
               const std::vector<t_xyz>&  Pw,
               const double               u,
               const int                  k,
               const int                  s,
               const int                  r,
               int&                       nq,
               std::vector<double>&       UQ,
               std::vector<t_xyz>&        Qw) const;

  //! Implements knot insertion algorithm for surfaces (A5.3). The knot
  //! values can be inserted in U direction or V direction.
  //!
  //! \param[in]  np   index of the last pole in the original surface in U direction.
  //! \param[in]  p    surface degree in U direction.
  //! \param[in]  UP   original knot vector in U direction.
  //! \param[in]  mp   index of the last pole in the original surface in V direction.
  //! \param[in]  q    surface degree in V direction.
  //! \param[in]  VP   original knot vector in V direction.
  //! \param[in]  Pw   control net.
  //! \param[in]  dir  parametric direction (U or V).
  //! \param[in]  knot knot to insert.
  //! \param[in]  k    knot interval.
  //! \param[in]  s    original multiplicity.
  //! \param[in]  r    how many times to insert.
  //! \param[out] nq   index of the last pole in the resulting surface in U direction.
  //! \param[out] UQ   resulting knot vector in U direction.
  //! \param[out] mq   index of the last pole in the resulting surface in V direction.
  //! \param[out] VQ   resulting knot vector in V direction.
  //! \param[out] Qw   resulting control net.
  //!
  //! \return true in case of success, false -- otherwise.
  mobiusBSpl_EXPORT bool
    operator()(const int                                np,
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
               std::vector< std::vector<t_xyz> >&       Qw) const;

};

};

#endif
