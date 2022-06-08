//-----------------------------------------------------------------------------
// Created on: 14 December 2018
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

#ifndef geom_MakeBicubicBSurf_HeaderFile
#define geom_MakeBicubicBSurf_HeaderFile

// Geometry includes
#include <mobius/geom_BSplineSurface.h>
#include <mobius/geom_BSurfNk.h>

// Core includes
#include <mobius/core_OPERATOR.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! This operator constructs a bicubic B-spline surface using the passed
//! constraints for the surface points and partial derivatives. This
//! functionality is mainly intended for internal use.
class geom_MakeBicubicBSurf : public core_OPERATOR
{
public:

  //! Ctor accepting 16 constraints used to define the control points
  //! of a B-surface.
  //! \param[in] S00        positional constraint at `(u,v) = (0,0)`.
  //! \param[in] S01        positional constraint at `(u,v) = (0,1)`.
  //! \param[in] S10        positional constraint at `(u,v) = (1,0)`.
  //! \param[in] S11        positional constraint at `(u,v) = (1,1)`.
  //! \param[in] dS_du00    derivative constraint at `(u,v) = (0,0)`.
  //! \param[in] dS_du01    derivative constraint at `(u,v) = (0,1)`.
  //! \param[in] dS_du10    derivative constraint at `(u,v) = (1,0)`.
  //! \param[in] dS_du11    derivative constraint at `(u,v) = (1,1)`.
  //! \param[in] dS_dv00    derivative constraint at `(u,v) = (0,0)`.
  //! \param[in] dS_dv01    derivative constraint at `(u,v) = (0,1)`.
  //! \param[in] dS_dv10    derivative constraint at `(u,v) = (1,0)`.
  //! \param[in] dS_dv11    derivative constraint at `(u,v) = (1,1)`.
  //! \param[in] d2S_dudv00 derivative constraint at `(u,v) = (0,0)`.
  //! \param[in] d2S_dudv01 derivative constraint at `(u,v) = (0,1)`.
  //! \param[in] d2S_dudv10 derivative constraint at `(u,v) = (1,0)`.
  //! \param[in] d2S_dudv11 derivative constraint at `(u,v) = (1,1)`.
  //! \param[in] progress   progress notifier.
  //! \param[in] plotter    imperative plotter.
  mobiusGeom_EXPORT
    geom_MakeBicubicBSurf(const t_xyz&       S00,
                          const t_xyz&       S01,
                          const t_xyz&       S10,
                          const t_xyz&       S11,
                          const t_xyz&       dS_du00,
                          const t_xyz&       dS_du01,
                          const t_xyz&       dS_du10,
                          const t_xyz&       dS_du11,
                          const t_xyz&       dS_dv00,
                          const t_xyz&       dS_dv01,
                          const t_xyz&       dS_dv10,
                          const t_xyz&       dS_dv11,
                          const t_xyz&       d2S_dudv00,
                          const t_xyz&       d2S_dudv01,
                          const t_xyz&       d2S_dudv10,
                          const t_xyz&       d2S_dudv11,
                          core_ProgressEntry progress,
                          core_PlotterEntry  plotter);

public:

  //! Constructs the surface.
  //! \return true in case of success, false -- otherwise.
  mobiusGeom_EXPORT bool
    Perform();

public:

  //! \return resulting surface.
  const t_ptr<t_bsurf>& GetResult() const
  {
    return m_surf;
  }

protected:

  t_xyz m_S00;        //!< Positional constraint at `(u,v) = (0,0)`.
  t_xyz m_S01;        //!< Positional constraint at `(u,v) = (0,1)`.
  t_xyz m_S10;        //!< Positional constraint at `(u,v) = (1,0)`.
  t_xyz m_S11;        //!< Positional constraint at `(u,v) = (1,1)`.
  t_xyz m_dS_du00;    //!< Derivative constraint at `(u,v) = (0,0)`.
  t_xyz m_dS_du01;    //!< Derivative constraint at `(u,v) = (0,1)`.
  t_xyz m_dS_du10;    //!< Derivative constraint at `(u,v) = (1,0)`.
  t_xyz m_dS_du11;    //!< Derivative constraint at `(u,v) = (1,1)`.
  t_xyz m_dS_dv00;    //!< Derivative constraint at `(u,v) = (0,0)`.
  t_xyz m_dS_dv01;    //!< Derivative constraint at `(u,v) = (0,1)`.
  t_xyz m_dS_dv10;    //!< Derivative constraint at `(u,v) = (1,0)`.
  t_xyz m_dS_dv11;    //!< Derivative constraint at `(u,v) = (1,1)`.
  t_xyz m_d2S_dudv00; //!< Derivative constraint at `(u,v) = (0,0)`.
  t_xyz m_d2S_dudv01; //!< Derivative constraint at `(u,v) = (0,1)`.
  t_xyz m_d2S_dudv10; //!< Derivative constraint at `(u,v) = (1,0)`.
  t_xyz m_d2S_dudv11; //!< Derivative constraint at `(u,v) = (1,1)`.

  //! Constructed surface.
  t_ptr<t_bsurf> m_surf;

};

}

#endif
