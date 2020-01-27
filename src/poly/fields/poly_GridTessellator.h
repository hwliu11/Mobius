//-----------------------------------------------------------------------------
// Created on: 27 January 2020
//-----------------------------------------------------------------------------
// Copyright (c) 2020-present, Sergey Slyadnev
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

#ifndef poly_GridTessellator_HeaderFile
#define poly_GridTessellator_HeaderFile

// Poly includes
#include <mobius/poly_Tessellator.h>

// Core includes
#include <mobius/core_OPERATOR.h>
#include <mobius/core_XYZ.h>

namespace mobius {

//! \ingroup MOBIUS_POLY
//!
//! Base class for DDF reconstruction operators working on regular Cartesian
//! grids.
class poly_GridTessellator : public poly_Tessellator
{
public:

  //! Ctor with initialization.
  //! \param[in] Pmin      min corner of the function domain.
  //! \param[in] Pmax      max corner of the function domain.
  //! \param[in] numSlices num of slices to discretize the space to get a regular grid.
  //! \param[in] progress  progress notifier.
  //! \param[in] plotter   imperative plotter.
  mobiusPoly_EXPORT
    poly_GridTessellator(const t_xyz&       Pmin,
                         const t_xyz&       Pmax,
                         const int          numSlices = 128,
                         core_ProgressEntry progress  = nullptr,
                         core_PlotterEntry  plotter   = nullptr);

  //! Dtor.
  mobiusPoly_EXPORT virtual
    ~poly_GridTessellator();

protected:

  t_xyz  m_Pmin;        //!< Min corner of the function domain.
  t_xyz  m_Pmax;        //!< Max corner of the function domain.
  int    m_iNumSlices;  //!< Num of slices for space discretization.
  int    m_iNumSlicesX; //!< Num of slices along OX axis.
  int    m_iNumSlicesY; //!< Num of slices along OY axis.
  int    m_iNumSlicesZ; //!< Num of slices along OZ axis.
  double m_fGrainX;     //!< Grain size along OX axis.
  double m_fGrainY;     //!< Grain size along OY axis.
  double m_fGrainZ;     //!< Grain size along OZ axis.

};

}

#endif
