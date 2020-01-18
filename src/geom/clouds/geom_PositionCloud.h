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

//! \ingroup MOBIUS_GEOM
//!
//! Represents simple point cloud as a collection of points.
class geom_PositionCloud : public geom_PointCloud
{
// Construction & destruction:
public:

  //! Default ctor.
  mobiusGeom_EXPORT
    geom_PositionCloud();

  //! Ctor accepting points of the cloud.
  //! \param[in] pts point to populate the cloud with.
  mobiusGeom_EXPORT
    geom_PositionCloud(const std::vector<t_xyz>& pts);

  //! Ctor accepting the plain vector of coordinates. This plain vector will
  //! be converted to the vector of triples.
  //! \param[in] coords coordinates packed in a plain vector.
  mobiusGeom_EXPORT
    geom_PositionCloud(const std::vector<double>& coords);

  //! Dtor.
  mobiusGeom_EXPORT virtual
    ~geom_PositionCloud();

public:

  //! Calculates boundary box for the point cloud.
  //! \param[out] xMin min X.
  //! \param[out] xMax max X.
  //! \param[out] yMin min Y.
  //! \param[out] yMax max Y.
  //! \param[out] zMin min Z.
  //! \param[out] zMax max Z.
  mobiusGeom_EXPORT virtual void
    GetBounds(double& xMin, double& xMax,
              double& yMin, double& yMax,
              double& zMin, double& zMax) const override;

public:

  //! Adds new point to the cloud.
  //! \param[in] point point to add.
  mobiusGeom_EXPORT virtual void
    AddPoint(const t_xyz& point);

  //! Adds new point to the cloud.
  //! \param[in] x X coordinate of a point to add.
  //! \param[in] y Y coordinate of a point to add.
  //! \param[in] z Z coordinate of a point to add.
  mobiusGeom_EXPORT virtual void
    AddPoint(const double x,
             const double y,
             const double z);

  //! Returns number of points.
  //! \return number of points.
  mobiusGeom_EXPORT virtual int
    GetNumberOfPoints() const;

  //! Returns point with the given zero-based index.
  //! \param[in] idx 0-based index of point to access.
  //! \return requested point.
  mobiusGeom_EXPORT virtual const t_xyz&
    GetPoint(const int idx) const;

  //! Returns point with the given zero-based index.
  //! \param[in]  idx 0-based index of point to access.
  //! \param[out] x   X coordinate of a point.
  //! \param[out] y   Y coordinate of a point.
  //! \param[out] z   Z coordinate of a point.
  mobiusGeom_EXPORT virtual void
    GetPoint(const int idx,
             double&   x,
             double&   y,
             double&   z) const;

  //! Sets point cloud.
  //! \param[in] cloud points to set.
  mobiusGeom_EXPORT virtual void
    SetPoints(const std::vector<t_xyz>& cloud);

  //! Returns internal collection of points.
  //! \return const reference to the internal collection of points.
  mobiusGeom_EXPORT virtual const std::vector<t_xyz>&
    GetPoints() const;

  //! Cleans up the cloud.
  mobiusGeom_EXPORT virtual void
    Clear();

public:

  //! Reads position cloud stored in the input file with common XYZ format. That
  //! is, the file contains just coordinate triples without any additional
  //! structuring information.
  //! \param[in] filename file to read.
  //! \return true in case of success, false -- otherwise.
  mobiusGeom_EXPORT bool
    Load(const std::string& filename);

  //! Writes position cloud to file with given filename.
  //! \param[in] filename file to write into.
  //! \return true in case of success, false -- otherwise.
  mobiusGeom_EXPORT bool
    SaveAs(const std::string& filename) const;

protected:

  //! Actual collection of points.
  std::vector<t_xyz> m_cloud;

};

//! Handy alias for double type.
typedef geom_PositionCloud t_pcloud;

}

#endif
