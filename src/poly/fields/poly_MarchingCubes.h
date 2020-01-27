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
#include <mobius/poly_GridTessellator.h>
#include <mobius/poly_ImplicitFunc.h>

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
class poly_MarchingCubes : public poly_GridTessellator
{
public:

  //! Ctor with initialization.
  //! \param[in] func      implicit function defining the field.
  //! \param[in] numSlices num of slices to discretize the space to get a regular grid.
  //! \param[in] progress  progress notifier.
  //! \param[in] plotter   imperative plotter.
  mobiusPoly_EXPORT
    poly_MarchingCubes(const t_ptr<poly_ImplicitFunc>& func,
                       const int                       numSlices = 128,
                       core_ProgressEntry              progress  = nullptr,
                       core_PlotterEntry               plotter   = nullptr);

  //! Dtor.
  virtual ~poly_MarchingCubes() = default;

private:

  //! Performs marching cubes reconstruction.
  //! \param[in] isoValue function level to polygonize.
  //! \return true in case of success, false -- otherwise.
  mobiusPoly_EXPORT virtual bool
    perform(const double isoValue);

private:

  //! Returns the Cartesian coordinates of the voxel corner for the passed
  //! integer indices.
  //! \param[in] nx 0-based index of the X coordinate of the corner.
  //! \param[in] ny 0-based index of the Y coordinate of the corner.
  //! \param[in] nz 0-based index of the Z coordinate of the corner.
  //! \return coordinates of the corner.
  t_xyz getVoxelCorner(const int nx, const int ny, const int nz) const;

  //! Interpolates the intersection point between the given extremities
  //! based on the passed scalar values.
  //! \param[in] point1  first point.
  //! \param[in] point2  second point.
  //! \param[in] scalar1 first scalar.
  //! \param[in] scalar2 second scalar.
  //! \return interpolated point.
  t_xyz interpVertex(const t_xyz& point1,
                     const t_xyz& point2,
                     const double scalar1,
                     const double scalar2) const;

protected:

  t_ptr<poly_ImplicitFunc> m_func; //!< Implicit function defining the field.

};

}

#endif
