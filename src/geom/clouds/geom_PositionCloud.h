//-----------------------------------------------------------------------------
// Created on: 15 November 2013
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

#ifndef geom_PositionCloud_HeaderFile
#define geom_PositionCloud_HeaderFile

// Geometry includes
#include <mobius/geom_PointCloud.h>

// Core includes
#include <mobius/core_XYZ.h>

// STL includes
#include <vector>

namespace mobius {

//! Represents simple point cloud as a collection of points.
class geom_PositionCloud : public geom_PointCloud
{
// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_PositionCloud();

  mobiusGeom_EXPORT
    geom_PositionCloud(const std::vector<core_XYZ>& pts);

  mobiusGeom_EXPORT virtual
    ~geom_PositionCloud();

public:

  mobiusGeom_EXPORT virtual void
    Bounds(adouble& xMin, adouble& xMax,
           adouble& yMin, adouble& yMax,
           adouble& zMin, adouble& zMax) const;

public:

  mobiusGeom_EXPORT virtual void
    AddPoint(const core_XYZ& point);

  mobiusGeom_EXPORT virtual size_t
    NumberOfPoints() const;

  mobiusGeom_EXPORT virtual const core_XYZ&
    Point(const size_t idx) const;

  mobiusGeom_EXPORT virtual void
    SetPoints(const std::vector<core_XYZ>& cloud);

  mobiusGeom_EXPORT virtual const std::vector<core_XYZ>&
    Points() const;

  mobiusGeom_EXPORT virtual void
    Clear();

protected:

  //! Actual collection of points.
  std::vector<core_XYZ> m_cloud;

};

//! Handy alias for adouble type.
typedef geom_PositionCloud pcloud;

};

#endif
