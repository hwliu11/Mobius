//-----------------------------------------------------------------------------
// Created on: 13 October 2013
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

#ifndef geom_SectionCloud_HeaderFile
#define geom_SectionCloud_HeaderFile

// Geometry includes
#include <mobius/geom_SectionLine.h>

// Core includes
#include <mobius/core_XYZ.h>

// STL includes
#include <vector>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Represents point cloud arranged as ordered sections of points.
class geom_SectionCloud : public geom_PointCloud
{
// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_SectionCloud();

  mobiusGeom_EXPORT
    geom_SectionCloud(const std::vector< t_ptr<geom_SectionLine> >& sections);

  mobiusGeom_EXPORT
    geom_SectionCloud(const std::vector< std::vector<t_xyz> >& sections);

  mobiusGeom_EXPORT virtual
    ~geom_SectionCloud();

public:

  mobiusGeom_EXPORT virtual void
    GetBounds(double& xMin, double& xMax,
              double& yMin, double& yMax,
              double& zMin, double& zMax) const override;

public:

  mobiusGeom_EXPORT void
    AddSection(const t_ptr<geom_SectionLine>& section);

  mobiusGeom_EXPORT size_t
    GetNumberOfSections() const;

  mobiusGeom_EXPORT const t_ptr<geom_SectionLine>&
    GetSectionByIndex(const size_t idx) const;

  mobiusGeom_EXPORT t_ptr<geom_SectionLine>
    GetSectionByID(const int ID) const;

  mobiusGeom_EXPORT const std::vector< t_ptr<geom_SectionLine> >&
    GetSections() const;

  mobiusGeom_EXPORT std::vector< std::vector<t_xyz> >
    GetPoints() const;

  mobiusGeom_EXPORT t_ptr<t_pcloud>
    ToPositionCloud() const;

private:

  //! Actual collection of points distributed by sections.
  std::vector< t_ptr<geom_SectionLine> > m_cloud;

};

//! Handy shortcut.
typedef geom_SectionCloud t_scloud;

};

#endif
