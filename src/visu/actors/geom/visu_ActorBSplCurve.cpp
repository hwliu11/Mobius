//-----------------------------------------------------------------------------
// Created on: 02 August 2013
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
#include <mobius/visu_ActorBSplCurve.h>

// geom includes
#include <mobius/geom_PointOnLine.h>

//! Constructor.
//! \param Crv                 [in] B-spline curve to draw.
//! \param CurveColor          [in] color of curve.
//! \param isPoints            [in] indicates whether to render curve with points.
//! \param noPolygon           [in] indicates whether to render the control polygon.
//! \param colorizeDerivatives [in] indicates whether derivatives have to be
//!                                 colorized.
mobius::visu_ActorBSplCurve::visu_ActorBSplCurve(const t_ptr<t_bcurve>&        Crv,
                                                 const visu_ColorRGB<GLubyte>& CurveColor,
                                                 const bool                    isPoints,
                                                 const bool                    noPolygon,
                                                 const bool                    colorizeDerivatives)
: visu_ActorInsensitive()
{
  m_curve                = Crv;
  m_curveColor           = CurveColor;
  m_bAsPoints            = isPoints;
  m_bNoPoly              = noPolygon;
  m_bColorizeDerivatives = colorizeDerivatives;
  m_fD1Min               = 0.0;
  m_fD1Max               = 0.0;
  m_bHedgehog            = false;

  double D1Avg = 0.0;
  if ( m_bColorizeDerivatives )
  {
    visu_ColorSelector::EstimateDerivative(Crv, m_fD1Min, m_fD1Max, D1Avg);

    std::cout << "Derivative delta: " << (m_fD1Max - m_fD1Min) << std::endl;
    std::cout << "Average derivative: " << D1Avg << std::endl;
  }

  //-------------------------------------------
  // Pre-evaluate curve for better performance
  //-------------------------------------------

  m_pDN = m_alloc.Allocate(Crv->GetDegree() + 1, Crv->GetDegree() + 1, false);

  const int    nPts = 1000;
  const double uMin = m_curve->GetMinParameter();
  const double uMax = m_curve->GetMaxParameter();
  const double step = (uMax - uMin) / nPts;

  // TODO: use some meaningful discretizer here
  for ( int pnt_idx = 0; pnt_idx <= nPts; ++pnt_idx )
  {
    double u = uMin + pnt_idx*step;
    if ( fabs(u - uMax) < 1.0e-10 )
      u = uMax; // Just to compensate round off errors

    t_xyz C, d1C;
    m_curve->Eval(u, C);
    m_curve->Eval_Dk(m_pDN, u, 1, d1C);

    // Evaluate curvature on curve
    if ( m_curve->GetDegree() >= 2 )
    {
      t_xyz d2C;
      const double K = m_curve->K(u);
      m_curve->Eval_Dk(u, 2, d2C);
      t_xyz planar_norm = d2C.Normalized();
      planar_norm = planar_norm * K * 10; // Factor for better visualization
      t_xyz C_norm = C + planar_norm;
      m_curvatures.push_back(C+planar_norm/*C_norm*/);
    }

    m_points.push_back(C);
    m_derivatives.push_back(d1C);
  }
}

//! Destructor.
mobius::visu_ActorBSplCurve::~visu_ActorBSplCurve()
{
}

//! Calculates boundary box for this Actor.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::visu_ActorBSplCurve::GetBounds(double& xMin, double& xMax,
                                            double& yMin, double& yMax,
                                            double& zMin, double& zMax) const
{
  m_curve->GetBounds(xMin, xMax, yMin, yMax, zMin, zMax);
}

//! Draws B-spline curve 3D.
void mobius::visu_ActorBSplCurve::GL_Draw()
{
  const std::vector<t_xyz>& poles = m_curve->GetPoles();

  /* ========================
   *  Render control polygon
   * ======================== */

  glEnable(GL_LINE_SMOOTH);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

    if ( !m_bNoPoly )
    {
      glPushAttrib(GL_ENABLE_BIT);
      glLineStipple(4, 0xAAAA);
      glEnable(GL_LINE_STIPPLE);
      glLineWidth(1);

      glBegin(GL_LINES);

        glColor3ub(QrWhite.R, QrWhite.G, QrWhite.B);
        t_xyz PolePrev = poles[0];
        for ( int p = 1; p < (int) poles.size(); ++p )
        {
          const t_xyz& Pole = poles[p];
          glVertex3f( (GLfloat) PolePrev.X(), (GLfloat) PolePrev.Y(), (GLfloat) PolePrev.Z() );
          glVertex3f( (GLfloat) Pole.X(), (GLfloat) Pole.Y(), (GLfloat) Pole.Z() );
          PolePrev = Pole;
        }

      glEnd();

      glDisable(GL_LINE_STIPPLE);
      glPopAttrib();
    }

  glDisable(GL_LINE_SMOOTH);

  /* ==================================================
   *  Render curve either with points or with segments
   * ================================================== */

  if ( m_bAsPoints )
  {
    glEnable(GL_POINT_SMOOTH);
      glPointSize(2);
      glBegin(GL_POINTS);
      glColor3ub(m_curveColor.R, m_curveColor.G, m_curveColor.B);

        const double uMin = m_curve->GetMinParameter();
        const double uMax = m_curve->GetMaxParameter();
        const double step = (uMax - uMin) / 100.0;
        double u = uMin;

        while ( u <= uMax )
        {
          t_xyz C;
          m_curve->Eval(u, C);
          glVertex3f( (GLfloat) C.X(), (GLfloat) C.Y(), (GLfloat) C.Z() );
          u += step;
        }

      glEnd();
    glDisable(GL_POINT_SMOOTH);
  }
  else
  {
    glEnable(GL_LINE_SMOOTH);
      glLineWidth(2);
      glBegin(GL_LINES);
      glColor3ub(m_curveColor.R, m_curveColor.G, m_curveColor.B);

        t_xyz C_prev = m_points[0];
        for ( size_t pt_idx = 1; pt_idx < m_points.size(); ++pt_idx )
        {
          const t_xyz& C = m_points[pt_idx];
          const t_xyz& d1C = m_derivatives[pt_idx];

          // Colorize curve points according to derivative values
          if ( m_bColorizeDerivatives )
          {
            visu_ColorRGB<GLubyte> der_color;
            visu_ColorSelector::ColorByDerivative_d(d1C.Modulus(), m_fD1Min, m_fD1Max, der_color);
            glColor3ub(der_color.R, der_color.G, der_color.B);
          }

          glVertex3f( (GLfloat) C_prev.X(), (GLfloat) C_prev.Y(), (GLfloat) C_prev.Z() );
          glVertex3f( (GLfloat) C.X(), (GLfloat) C.Y(), (GLfloat) C.Z() );

          C_prev = C;
        }

      glEnd();
      glLineWidth(1);
    glDisable(GL_LINE_SMOOTH);

    //----------------------------
    // Visualize hedgehog diagram
    //----------------------------

    if ( m_bHedgehog )
    {
      glLineWidth(1);
      glBegin(GL_LINES);

        t_xyz K_prev;
        bool no_envelope = true;
        for ( size_t k = 0; k < m_points.size(); ++k )
        {
          const t_xyz& C = m_points[k];
          const t_xyz& K = m_curvatures[k];

          if ( fabs( (K - C).Modulus() ) > visu_KC_limit_min )
          {
            glColor3ub(QrRed.R, QrRed.G, QrRed.B);
            glVertex3f( (GLfloat) C.X(), (GLfloat) C.Y(), (GLfloat) C.Z() );
            glVertex3f( (GLfloat) K.X(), (GLfloat) K.Y(), (GLfloat) K.Z() );

            if ( !no_envelope )
            {
              glColor3ub(QrWhite.R, QrWhite.G, QrWhite.B);
              glVertex3f( (GLfloat) K_prev.X(), (GLfloat) K_prev.Y(), (GLfloat) K_prev.Z() );
              glVertex3f( (GLfloat) K.X(), (GLfloat) K.Y(), (GLfloat) K.Z() );
            }

            K_prev = K;
            no_envelope = false;
          }
          else
            no_envelope = true;
        }
      glEnd();
    }
  }

 /* ==============
  *  Render poles
  * ============== */

  if ( !m_bNoPoly )
  {
    glEnable(GL_POINT_SMOOTH);
    glBegin(GL_POINTS);
    glPointSize(3);

      glColor3ub(QrWhite.R, QrWhite.G, QrWhite.B);

      for ( int p = 0; p < (int) poles.size(); ++p )
      {
        const t_xyz& Pole = poles[p];
        glVertex3f( (GLfloat) Pole.X(), (GLfloat) Pole.Y(), (GLfloat) Pole.Z() );
      }

      // TODO remove this
      glPointSize(10);
      glColor3f(1.0f, 0.0f, 0.0f);
      glVertex3f( (GLfloat) m_hiliPole.X(), (GLfloat) m_hiliPole.Y(), (GLfloat) m_hiliPole.Z() );

    glEnd();
    glDisable(GL_POINT_SMOOTH);
  }
}
