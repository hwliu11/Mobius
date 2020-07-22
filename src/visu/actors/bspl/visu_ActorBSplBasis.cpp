//-----------------------------------------------------------------------------
// Created on: 19 July 2013
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
#include <mobius/visu_ActorBSplBasis.h>

// visu includes
#include <mobius/visu_ColorSelector.h>

// bspl includes
#include <mobius/bspl_NDiscrete.h>

//! Constructor.
//! \param U   [in] knot vector.
//! \param deg [in] degree of B-spline basis functions to draw.
mobius::visu_ActorBSplBasis::visu_ActorBSplBasis(const std::vector<double>& U,
                                                 const int                  deg)
: visu_ActorInsensitive(),
  m_U(U),
  m_iDeg(deg)
{
  //-----------------------------------------------
  // Pre-evaluate functions for better performance
  //-----------------------------------------------

  const int m  = (int) m_U.size() - 1;
  const int nN = m - 1;
  //
  for ( int i = 0; i <= nN; ++i )
  {
    std::vector<t_xyz> func_pts;
    bspl_NDiscrete NDiscrete(U, m_iDeg);
    NDiscrete.Perform(i, 0.001,
                      bspl_NDiscrete::Strategy_UniformAbscissa);
    if ( NDiscrete.IsDone() )
    {
      const std::vector<double>& X = NDiscrete.Abscissa();
      const std::vector<double>& Y = NDiscrete.Values();

      for ( size_t j = 0; j < X.size(); ++j )
        func_pts.push_back( t_xyz(X[j], Y[j], 0.0) );
    }
    m_pts.push_back(func_pts);
  }
}

//! Destructor.
mobius::visu_ActorBSplBasis::~visu_ActorBSplBasis()
{}

//! Calculates boundary box for this Actor.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::visu_ActorBSplBasis::GetBounds(double& xMin, double& xMax,
                                            double& yMin, double& yMax,
                                            double& zMin, double& zMax) const
{
  xMin = 0.0;
  xMax = m_U[m_U.size() - 1];
  yMin = 0.0;
  yMax = xMax;
  zMin = zMax = 0.0;
}

//! Draws B-spline basis functions.
void mobius::visu_ActorBSplBasis::GL_Draw()
{
  #pragma region GL rendering
  glEnable(GL_LINE_SMOOTH);
    glLineWidth(2.0f);

    for ( size_t f = 0; f < m_pts.size(); ++f )
    {
      glBegin(GL_LINES);
        glColor3ub(QrWhite.R, QrWhite.G, QrWhite.B);
        for ( size_t p = 0; p < m_pts[f].size() - 1; ++p )
        {
          glVertex3f( (GLfloat) m_pts[f][p].X(),     (GLfloat) m_pts[f][p].Y(),     (GLfloat) m_pts[f][p].Z() );
          glVertex3f( (GLfloat) m_pts[f][p + 1].X(), (GLfloat) m_pts[f][p + 1].Y(), (GLfloat) m_pts[f][p + 1].Z() );
        }
      glEnd();
    }

    glLineWidth(1.0f);
  glDisable(GL_LINE_SMOOTH);
  #pragma endregion
}
