//-----------------------------------------------------------------------------
// Created on: 25 June 2021
//-----------------------------------------------------------------------------
// Copyright (c) 2021-present, Sergey Slyadnev
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

#ifndef core_BBox_HeaderFile
#define core_BBox_HeaderFile

// Core includes
#include <mobius/core_XYZ.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Axis-aligned bounding box.
struct core_BBox
{
  bool  IsVoid; //!< Am I empty?
  t_xyz minPt;  //!< Min corner point.
  t_xyz maxPt;  //!< Max corner point.

  core_BBox() : IsVoid(true) {} //!< Default ctor.

  mobiusCore_EXPORT void
    Get(double& xmin, double& ymin, double& zmin,
        double& xmax, double& ymax, double& zmax) const;

  mobiusCore_EXPORT void
    Add(const t_xyz& P);

  mobiusCore_EXPORT void
    Add(const double x, const double y, const double z);

  mobiusCore_EXPORT bool
    IsOut(const t_xyz& P,
          const double tolerance) const;

  mobiusCore_EXPORT bool
    IsOut(const core_BBox& other,
          const double     tolerance) const;

};

}

#endif
