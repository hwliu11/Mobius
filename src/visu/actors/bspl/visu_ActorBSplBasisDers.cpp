//-----------------------------------------------------------------------------
// Created on: 15 February 2013
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

// Own include
#include <mobius/visu_ActorBSplBasisDers.h>

// visu includes
#include <mobius/visu_ColorSelector.h>

// bspl includes
#include <mobius/bspl_EffectiveNDers.h>
#include <mobius/bspl_FindSpan.h>

//! Constructor.
//! \param U   [in] knots.
//! \param deg [in] degree of B-spline basis functions to draw.
//! \param k   [in] order of derivatives.
mobius::visu_ActorBSplBasisDers::visu_ActorBSplBasisDers(const std::vector<double>& U,
                                                         const int                  deg,
                                                         const int                  k)
: visu_ActorInsensitive(),
  m_U(U),
  m_iDeg(deg),
  m_iOrder(k)
{
  // Array to store evaluated function/derivative values
  m_pF = m_alloc2D.Allocate(m_iDeg+1, m_iDeg+1, false);
}

//! Destructor.
mobius::visu_ActorBSplBasisDers::~visu_ActorBSplBasisDers()
{}

//! Calculates boundary box for this Actor.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::visu_ActorBSplBasisDers::GetBounds(double& xMin, double& xMax,
                                                double& yMin, double& yMax,
                                                double& zMin, double& zMax) const
{
  xMin = 0.0;
  xMax = m_U[m_U.size() - 1];
  yMin = 0.0;
  yMax = 1.0;
  zMin = zMax = 0.0;
}

//! Draws B-spline basis functions.
void mobius::visu_ActorBSplBasisDers::GL_Draw()
{
  // TODO: draw something more useful
  bspl_EffectiveNDers NDers;
  bspl_FindSpan FindSpan(m_U, m_iDeg);

  // Color for functions
  visu_ColorRGB<GLfloat> rgb(1.0f, 0.0f, 0.0f);

  #pragma region GL rendering
  glEnable(GL_POINT_SMOOTH);
    glPointSize(1);
    glBegin(GL_POINTS);

      const double u_min  = m_U[0];
      const double u_max  = m_U[m_U.size() - 1];
      const double u_step = (u_max - u_min) / 500.0;
      double       u      = u_min;

      while ( u <= u_max )
      {
        int span_idx = FindSpan(u);
        NDers(u, m_U, m_iDeg, span_idx, m_iOrder, m_pF);

        for ( int r = 0; r <= m_iDeg; ++r )
        {
          // Draw
          glColor3f(rgb.R, rgb.G, rgb.B);
          glVertex3f(3 * (GLfloat) u, 3 * (GLfloat) m_pF[m_iOrder][r], 0);

        }
        u += u_step;
      }

    glEnd();
  glDisable(GL_POINT_SMOOTH);
  #pragma endregion
}
