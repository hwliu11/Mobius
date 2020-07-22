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

// Windows
#include <windows.h>

// GL includes
#include <gl/gl.h>
#include <gl/glu.h>

// Own include
#include <mobius/visu_ActorVectorField.h>

// core includes
#include <mobius/core_Precision.h>

//! Constructor.
//! \param Field [in] vector field to draw.
//! \param Color [in] pixel color to use for all points in the cloud.
//! \param max_modulus [in] max modulus of a vector. If specified, the vector
//!        field will be rescaled so that to have the biggest vector with
//!        modulus equal to max. If not specified, no rescaling occurs.
mobius::visu_ActorVectorField::visu_ActorVectorField(const t_ptr<geom_VectorField>& Field,
                                                     const visu_ColorRGB<GLubyte>&  Color,
                                                     const double                   max_modulus)
: visu_ActorInsensitive()
{
  m_field               = Field;
  m_color               = Color;
  m_fMaxModulus_desired = max_modulus;
  m_fMaxModulus_inField = 0.0;

  // Find max modulus in the field
  for ( int p = 0; p < m_field->GetCloud()->GetNumberOfPoints(); ++p )
  {
    t_xyz vector = m_field->GetVector(p);
    if ( vector.IsOrigin() )
      continue;

    m_fMaxModulus_inField = max( m_fMaxModulus_inField, vector.Modulus() );
  }
}

//! Destructor.
mobius::visu_ActorVectorField::~visu_ActorVectorField()
{
}

//! Calculates boundary box for this Actor.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::visu_ActorVectorField::GetBounds(double& xMin, double& xMax,
                                              double& yMin, double& yMax,
                                              double& zMin, double& zMax) const
{
  m_field->GetCloud()->GetBounds(xMin, xMax, yMin, yMax, zMin, zMax);
}

//! Draws vector field.
void mobius::visu_ActorVectorField::GL_Draw()
{
  glEnable(GL_LINE_SMOOTH);
  glLineWidth(3);
  glBegin(GL_LINES);
  glColor3ub(m_color.R, m_color.G, m_color.B);

    for ( int p = 0; p < m_field->GetCloud()->GetNumberOfPoints(); ++p )
    {
      t_xyz vector = m_field->GetVector(p);
      if ( vector.IsOrigin( core_Precision::Resolution3D() ) )
        continue;

      if ( m_fMaxModulus_desired ) // rescale if requested
        vector = vector / m_fMaxModulus_inField * m_fMaxModulus_desired;

      const t_xyz& start = m_field->GetCloud()->GetPoint(p);
      t_xyz        end   = start + vector;

      glVertex3f( (GLfloat) start.X(), (GLfloat) start.Y(), (GLfloat) start.Z() );
      glVertex3f( (GLfloat) end.X(), (GLfloat) end.Y(), (GLfloat) end.Z() );
    }

  glEnd();
  glDisable(GL_LINE_SMOOTH);
}
