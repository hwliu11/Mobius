//-----------------------------------------------------------------------------
// Created on: 11 August 2013
//-----------------------------------------------------------------------------
// Copyright (c) 2013-present, Sergey Slyadnev
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

#ifndef visu_ActorCartesianAxes_HeaderFile
#define visu_ActorCartesianAxes_HeaderFile

// visu includes
#include <mobius/visu_ActorInsensitive.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Class representing 3D cartesian axes.
class visu_ActorCartesianAxes : public visu_ActorInsensitive
{
public:

  mobiusVisu_EXPORT
    visu_ActorCartesianAxes(const double axisLength = 1.0);

  mobiusVisu_EXPORT virtual
    ~visu_ActorCartesianAxes();

public:

  mobiusVisu_EXPORT virtual void
    GetBounds(double& xMin, double& xMax,
              double& yMin, double& yMax,
              double& zMin, double& zMax) const;

public:

  mobiusVisu_EXPORT virtual void
    GL_Draw();

public:

  mobiusVisu_EXPORT virtual void
    SetAxisLength(const double axisLength);

  mobiusVisu_EXPORT virtual double
    AxisLength() const;

private:

  double m_fAxisLength; //!< Length of each axis.

};

}

#endif
