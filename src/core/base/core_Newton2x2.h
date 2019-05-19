//-----------------------------------------------------------------------------
// Created on: 17 May 2019
//-----------------------------------------------------------------------------
// Copyright (c) 2019-present, Sergey Slyadnev
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

#ifndef core_Newton2x2_HeaderFile
#define core_Newton2x2_HeaderFile

// Core includes
#include <mobius/core_IAlgorithm.h>
#include <mobius/core_TwovariateFuncWithGradient.h>
#include <mobius/core_UV.h>

#define COUT_DEBUG
#if defined COUT_DEBUG
  #pragma message("===== warning: COUT_DEBUG is enabled")
#endif

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Newton method for solving `F(x) = 0` where `F = (f, g)` is a two-dimensional
//! function of two variables `u` and `v` (hence the dimension of the problem
//! is 2x2).
class core_Newton2x2 : public core_IAlgorithm
{
public:

  //! Ctor.
  //! \param[in] f        first component of the objective vector function.
  //! \param[in] g        second component of the objective vector function.
  //! \param[in] progress progress notifier.
  //! \param[in] plotter  imperative plotter.
  core_Newton2x2(const core_Ptr<core_TwovariateFuncWithGradient>& f,
                 const core_Ptr<core_TwovariateFuncWithGradient>& g,
                 core_ProgressEntry                               progress = NULL,
                 core_PlotterEntry                                plotter  = NULL)
  //
  : core_IAlgorithm (progress, plotter),
    m_f             (f),
    m_g             (g)
  {}

public:

  //! Performs Newton iterations starting from the passed initial guess.
  //! \param[in]  x0   two-dimensional initial guess.
  //! \param[in]  prec requested precision.
  //! \param[out] xsol found solution.
  //! \return true in case of success, false -- otherwise.
  bool Perform(const core_UV& x0,
               const double   prec,
               core_UV&       xsol) const
  {
    double    u_i     = x0.U();
    double    v_i     = x0.V();
    bool      stop    = false;
    bool      success = false;
    const int maxIter = 100;
    int       iter    = 0;

    do
    {
      // Compute functions f, g with their derivatives.
      double f_i, fu_i, fv_i, g_i, gu_i, gv_i;
      //
      m_f->EvalWithGrad(u_i, v_i, f_i, fu_i, fv_i);
      m_g->EvalWithGrad(u_i, v_i, g_i, gu_i, gv_i);

      // Check if the target precision is reached.
      if ( fabs(f_i) < prec && fabs(g_i) < prec )
      {
        stop    = true;
        success = true;
        xsol    = uv(u_i, v_i);
        continue;
      }

#if defined COUT_DEBUG
      std::cout << "f(" << u_i << ", " << v_i << ") = " << f_i << std::endl;
      std::cout << "g(" << u_i << ", " << v_i << ") = " << g_i << std::endl;
#endif

      // Compute next value of v.
      const double v_next = v_i + (gu_i*f_i - g_i*fu_i) / (fu_i*gv_i - gu_i*fv_i);

      // Compute next value of u.
      const double u_next = u_i - ( f_i + fv_i*(v_next - v_i) ) / fu_i;

      // Halt by exceeding the limit of iterations.
      if ( ++iter == maxIter )
      {
        stop = true;
        continue;
      }

      u_i = u_next;
      v_i = v_next;
    }
    while ( !stop );

    return success;
  }

private:

  core_Ptr<core_TwovariateFuncWithGradient> m_f; //!< First-component function.
  core_Ptr<core_TwovariateFuncWithGradient> m_g; //!< Second-component function.

};

};

#endif
