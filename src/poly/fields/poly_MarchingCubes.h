//-----------------------------------------------------------------------------
// Created on: 27 December 2019
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

#ifndef poly_MarchingCubes_HeaderFile
#define poly_MarchingCubes_HeaderFile

// Poly includes
#include <mobius/poly_ImplicitFunc.h>

// Core includes
#include <mobius/core_OPERATOR.h>

namespace mobius {

//! \ingroup MOBIUS_POLY
//!
//! This class provides implementation of the classical marching cubes
//! algorithm introduced in the seminal paper by W. Lorensen
//!
//!  [Lorensen, W.E. and Cline, H.E. 1987. Marching cubes: A high resolution
//!   3D surface construction algorithm. ACM SIGGRAPH Computer Graphics
//!   21, 4, 163-169]
//!
//! for the contouring of medical data. There is a plenty of open source
//! implementations of this algorithm, e.g., the one from VTK or the algorithm
//! by Paul Bourke
//!
//!  [Bourke, P. 1994. Polygonising a scalar field.
//!   URL: http://paulbourke.net/geometry/polygonise/].
//!
//! Our implementation is no different from the "standard" ones. It is
//! introduced here just to support the native data structures of the library,
//! hence avoiding conversion between different mesh representations.
//!
//! The marching cubes algorithm combines simplicity with high speed since
//! it works almost entirely on lookup tables. We use MC as a reconstruction
//! operator for our adaptive scalar fields (hosted by SVO nodes). MC is
//! used for creating a 3D contour of a mathematical scalar field. In this case,
//! the function is known everywhere but is sampled at the vertices of a regular
//! 3D grid.
class poly_MarchingCubes : public core_OPERATOR
{
public:

  //! Ctor with initialization.
  //! \param[in] func     implicit function defining the field.
  //! \param[in] grain    grain value to discretize the space to get a regular grid.
  //! \param[in] progress progress notifier.
  //! \param[in] plotter  imperative plotter.
  mobiusPoly_EXPORT
    poly_MarchingCubes(const t_ptr<poly_ImplicitFunc>& func,
                       const double                    grain,
                       core_ProgressEntry              progress = nullptr,
                       core_PlotterEntry               plotter  = nullptr);

  //! Dtor.
  mobiusPoly_EXPORT virtual
    ~poly_MarchingCubes();

public:

  //! Performs polygonal approximation.
  mobiusPoly_EXPORT bool
    Perform();

protected:

  t_ptr<poly_ImplicitFunc> m_func;   //!< Implicit function representing the field.
  double                   m_fGrain; //!< Space discretization grain.

};

}

#endif
