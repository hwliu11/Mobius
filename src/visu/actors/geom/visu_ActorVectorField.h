//-----------------------------------------------------------------------------
// Created on: 25 February 2015
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

#ifndef visu_ActorVectorField_HeaderFile
#define visu_ActorVectorField_HeaderFile

// visu includes
#include <mobius/visu_ActorInsensitive.h>
#include <mobius/visu_ColorSelector.h>

// geom includes
#include <mobius/geom_VectorField.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Class representing OpenGL Actor dedicated to visualization of Vector Field.
class visu_ActorVectorField : public visu_ActorInsensitive
{
public:

  mobiusVisu_EXPORT
    visu_ActorVectorField(const t_ptr<geom_VectorField>& Field,
                            const visu_ColorRGB<GLubyte>& Color,
                            const double max_modulus = 0.0);

  mobiusVisu_EXPORT virtual
    ~visu_ActorVectorField();

public:

  mobiusVisu_EXPORT virtual void
    GetBounds(double& xMin, double& xMax,
              double& yMin, double& yMax,
              double& zMin, double& zMax) const;

  mobiusVisu_EXPORT virtual void
    GL_Draw();

private:

  //! Vector field to draw.
  t_ptr<geom_VectorField> m_field;

  //! Color.
  visu_ColorRGB<GLubyte> m_color;

  //! Max modulus.
  double m_fMaxModulus_desired;

  //! Max modulus in the field.
  double m_fMaxModulus_inField;

};

}

#endif