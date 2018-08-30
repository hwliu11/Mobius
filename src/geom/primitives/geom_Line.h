//-----------------------------------------------------------------------------
// Created on: 22 May 2014
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

#ifndef geom_Line_HeaderFile
#define geom_Line_HeaderFile

// Geometry includes
#include <mobius/geom_Curve.h>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Parametric straight line.
class geom_Line : public geom_Curve
{
// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_Line(const core_XYZ& origin,
              const core_XYZ& dir);

  mobiusGeom_EXPORT virtual
    ~geom_Line();

public:

  mobiusGeom_EXPORT virtual void
    Bounds(adouble& xMin, adouble& xMax,
           adouble& yMin, adouble& yMax,
           adouble& zMin, adouble& zMax) const;

public:

  mobiusGeom_EXPORT virtual adouble
    MinParameter() const;

  mobiusGeom_EXPORT virtual adouble
    MaxParameter() const;

  mobiusGeom_EXPORT virtual void
    Eval(const adouble u,
         core_XYZ&    P) const;

public:

  //! Accessor for origin.
  //! \return origin of the line.
  const core_XYZ& Origin() const
  {
    return m_origin;
  }

  //! Accessor for direction.
  //! \return direction vector of the line.
  const core_XYZ& Dir() const
  {
    return m_dir;
  }

private:

  //! Origin of the line.
  core_XYZ m_origin;

  //! Direction of the line.
  core_XYZ m_dir;

};

};

#endif
