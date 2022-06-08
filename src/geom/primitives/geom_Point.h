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

#ifndef geom_Point_HeaderFile
#define geom_Point_HeaderFile

// Geometry includes
#include <mobius/geom_Geometry.h>

// Core includes
#include <mobius/core_XYZ.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Geometric primitive for a shared 3D point.
class geom_Point : public geom_Geometry
{
// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_Point();

  mobiusGeom_EXPORT
    geom_Point(const double x,
               const double y,
               const double z);

  mobiusGeom_EXPORT
    geom_Point(const geom_Point& PP);

  mobiusGeom_EXPORT virtual
    ~geom_Point();

public:

  mobiusGeom_EXPORT virtual void
    GetBounds(double& xMin, double& xMax,
              double& yMin, double& yMax,
              double& zMin, double& zMax) const override;

public:

  //! Returns X co-ordinate of the 3D point.
  //! \return X co-ordinate.
  double X() const
  {
    return m_XYZ.X();
  }

  //! Returns Y co-ordinate of the 3D point.
  //! \return Y co-ordinate.
  double Y() const
  {
    return m_XYZ.Y();
  }

  //! Returns Z co-ordinate of the 3D point.
  //! \return Z co-ordinate.
  double Z() const
  {
    return m_XYZ.Z();
  }

public:

  mobiusGeom_EXPORT geom_Point
    Multiplied(const double coeff) const;

public:

  mobiusGeom_EXPORT geom_Point&
    operator=(const geom_Point& PP);

  mobiusGeom_EXPORT geom_Point
    operator*(const double coeff) const;

  mobiusGeom_EXPORT geom_Point
    operator+(const geom_Point& PP) const;

  mobiusGeom_EXPORT geom_Point&
    operator+=(const geom_Point& PP);

  mobiusGeom_EXPORT geom_Point&
    operator+=(const t_ptr<geom_Point>& hPP);

  mobiusGeom_EXPORT geom_Point
    operator-(const geom_Point& PP) const;

  mobiusGeom_EXPORT geom_Point&
    operator-=(const geom_Point& PP);

  mobiusGeom_EXPORT geom_Point&
    operator-=(const t_ptr<geom_Point>& hPP);

private:

  t_xyz m_XYZ; //!< XYZ primitive.

};

}

#endif
