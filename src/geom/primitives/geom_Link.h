//-----------------------------------------------------------------------------
// Created on: 23 May 2013
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

#ifndef geom_Link_HeaderFile
#define geom_Link_HeaderFile

// Geometry includes
#include <mobius/geom_Geometry.h>
#include <mobius/geom_Point.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Represents simple oriented link between two 3D points.
//!
//! \todo complete description
class geom_Link : public geom_Geometry
{
// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_Link(const t_xyz& P1,
              const t_xyz& P2);

  mobiusGeom_EXPORT virtual
    ~geom_Link();

public:

  mobiusGeom_EXPORT virtual void
    GetBounds(double& xMin, double& xMax,
              double& yMin, double& yMax,
              double& zMin, double& zMax) const override;

public:

  //! Returns the first connection point.
  //! \return first point.
  const t_xyz& P1() const
  {
    return m_p1;
  }

  //! Returns the second connection point.
  //! \return second point.
  const t_xyz& P2() const
  {
    return m_p2;
  }

private:

  t_xyz m_p1; //!< First point.
  t_xyz m_p2; //!< Second point.

};

}

#endif
