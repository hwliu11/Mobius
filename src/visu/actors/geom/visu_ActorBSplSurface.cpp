//-----------------------------------------------------------------------------
// Created on: 07 February 2014
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
#include <mobius/visu_ActorBSplSurface.h>

// visu includes
#include <mobius/visu_ColorSelector.h>

//! Constructor.
//! \param Surf [in] B-spline surface to draw.
mobius::visu_ActorBSplSurface::visu_ActorBSplSurface(const t_ptr<t_bsurf>& Surf)
//
: visu_ActorInsensitiveSurface ( Surf.Access() ),
  m_bHedgehog_U                ( false ),
  m_bHedgehog_V                ( false ),
  m_bNoPoly                    ( true )
{
  t_ptr<t_bsurf> bSurf = t_ptr<t_bsurf>::DownCast(m_surf);

  const int stepsNumber = 25;

  /* ============================================
   *  Fill properties for U-isoparametric curves
   * ============================================ */

  const double uMin = m_surf->GetMinParameter_U();
  const double uMax = m_surf->GetMaxParameter_U();
  const double uStep = (uMax - uMin) / stepsNumber;

  double u = uMin;
  bool uStop = false;
  while ( !uStop )
  {
    if ( u > uMax )
    {
      u = uMax;
      uStop = true;
    }

    t_ptr<t_bcurve> Iso = bSurf->Iso_U(u);
    this->fillIsolineProps(Iso, m_isoU, m_isoU_N);

    u += uStep;
  }

  /* ============================================
   *  Fill properties for V-isoparametric curves
   * ============================================ */

  const double vMin = m_surf->GetMinParameter_V();
  const double vMax = m_surf->GetMaxParameter_V();
  const double vStep = (vMax - vMin) / stepsNumber;

  double v = vMin;
  bool vStop = false;
  while ( !vStop )
  {
    if ( v > vMax )
    {
      v = vMax;
      vStop = true;
    }

    t_ptr<t_bcurve> Iso = bSurf->Iso_V(v);
    this->fillIsolineProps(Iso, m_isoV, m_isoV_N);

    v += vStep;
  }
}

//! Destructor.
mobius::visu_ActorBSplSurface::~visu_ActorBSplSurface()
{
}

//! Draws B-spline surface.
void mobius::visu_ActorBSplSurface::GL_Draw()
{
  t_ptr<t_bsurf> bSurf = t_ptr<t_bsurf>::DownCast(m_surf);

  /* ========================
   *  Render control polygon
   * ======================== */

  if ( !m_bNoPoly )
  {
    const std::vector< std::vector<t_xyz> >& poles = bSurf->GetPoles();

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glPushAttrib(GL_ENABLE_BIT);
    glLineStipple(4, 0xAAAA);
    glEnable(GL_LINE_STIPPLE);
    glLineWidth(1);

    glBegin(GL_LINES);

      glColor3ub(QrWhite.R, QrWhite.G, QrWhite.B);

      for ( int i = 0; i < (int) poles.size(); ++i )
      {
        t_xyz PolePrev = poles[i][0];
        for ( int j = 1; j < (int) poles[i].size(); ++j )
        {
          const t_xyz& Pole = poles[i][j];
          glVertex3f( (GLfloat) PolePrev.X(), (GLfloat) PolePrev.Y(), (GLfloat) PolePrev.Z() );
          glVertex3f( (GLfloat) Pole.X(), (GLfloat) Pole.Y(), (GLfloat) Pole.Z() );
          PolePrev = Pole;
        }
      }

      for ( int j = 0; j < (int) poles[0].size(); ++j )
      {
        t_xyz PolePrev = poles[0][j];
        for ( int i = 1; i < (int) poles.size(); ++i )
        {
          const t_xyz& Pole = poles[i][j];
          glVertex3f( (GLfloat) PolePrev.X(), (GLfloat) PolePrev.Y(), (GLfloat) PolePrev.Z() );
          glVertex3f( (GLfloat) Pole.X(), (GLfloat) Pole.Y(), (GLfloat) Pole.Z() );
          PolePrev = Pole;
        }
      }

    glEnd();

    glDisable(GL_LINE_STIPPLE);
    glPopAttrib();
    glDisable(GL_LINE_SMOOTH);

    // --------------
    //  Render poles
    // --------------

    glPointSize(5);
    glBegin(GL_POINTS);

      glColor3ub(QrWhite.R, QrWhite.G, QrWhite.B);

      for ( int i = 0; i < (int) poles.size(); ++i )
      {
        for ( int j = 0; j < (int) poles[i].size(); ++j )
        {
          const t_xyz& Pole = poles[i][j];
          glVertex3f( (GLfloat) Pole.X(), (GLfloat) Pole.Y(), (GLfloat) Pole.Z() );
        }
      }

    glEnd();
  }

  /* ==========================
   *  Render points on surface
   * ========================== */

  #pragma region Filling
  //glEnable(GL_POINT_SMOOTH);
  //glPointSize(1);

  //  glColor3f(0.0f, 0.0f, 1.0f);
  //  glEnableClientState(GL_VERTEX_ARRAY);
  //    glVertexPointer(3, GL_FLOAT, 0, m_pFilling);
  //    glDrawArrays(GL_POINTS, 0, m_iNumFillingPoints);
  //  glDisableClientState(GL_VERTEX_ARRAY);

  //glEnd();
  //glDisable(GL_POINT_SMOOTH);
  #pragma endregion

  glEnable(GL_LINE_SMOOTH);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

  /* ===============================
   *  Render U-isoparametric curves
   * =============================== */

  glLineWidth(1.5f);
  glBegin(GL_LINES);
    glColor3ub(QrGray.R, QrGray.G, QrGray.B);
    for ( size_t iso = 0; iso < m_isoU.size(); ++iso )
    {
      const std::vector<t_xyz>& points = m_isoU[iso];
      t_xyz C_prev = points[0];
      for ( size_t k = 1; k < points.size(); ++k )
      {
        const t_xyz& C = points[k];
        glVertex3f( (GLfloat) C_prev.X(), (GLfloat) C_prev.Y(), (GLfloat) C_prev.Z() );
        glVertex3f( (GLfloat) C.X(), (GLfloat) C.Y(), (GLfloat) C.Z() );
        C_prev = C;
      }
    }
  glEnd();

  if ( m_bHedgehog_U )
  {
    glLineWidth(1);
    glBegin(GL_LINES);
      for ( size_t iso = 0; iso < m_isoU.size(); ++iso )
      {
        const std::vector<t_xyz>& points = m_isoU[iso];
        const std::vector<t_xyz>& curvatures = m_isoU_N[iso];

        if ( !curvatures.size() )
          continue;

        t_xyz K_prev;
        bool no_envelope = true;
        for ( size_t k = 0; k < points.size(); ++k )
        {
          const t_xyz& C = points[k];
          const t_xyz& K = curvatures[k];

          if ( fabs( (K - C).Modulus() ) > visu_KS_limit_min )
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
      }
    glEnd();
  }

  /* ===============================
   *  Render V-isoparametric curves
   * =============================== */

  glLineWidth(1.5f);
  glBegin(GL_LINES);
    glColor3ub(QrGray.R, QrGray.G, QrGray.B);
    for ( size_t iso = 0; iso < m_isoV.size(); ++iso )
    {
      const std::vector<t_xyz>& points = m_isoV[iso];
      t_xyz C_prev = points[0];
      for ( size_t k = 1; k < points.size(); ++k )
      {
        const t_xyz& C = points[k];
        glVertex3f( (GLfloat) C_prev.X(), (GLfloat) C_prev.Y(), (GLfloat) C_prev.Z() );
        glVertex3f( (GLfloat) C.X(), (GLfloat) C.Y(), (GLfloat) C.Z() );
        C_prev = C;
      }
    }
  glEnd();

  if ( m_bHedgehog_V )
  {
    glLineWidth(1);
    glBegin(GL_LINES);
      glColor3ub(QrGreen.R, QrGreen.G, QrGreen.B);
      for ( size_t iso = 0; iso < m_isoV.size(); ++iso )
      {
        const std::vector<t_xyz>& points = m_isoV[iso];
        const std::vector<t_xyz>& curvatures = m_isoV_N[iso];

        if ( !curvatures.size() )
          continue;

        t_xyz K_prev;
        bool no_envelope = true;
        for ( size_t k = 0; k < points.size(); ++k )
        {
          const t_xyz& C = points[k];
          const t_xyz& K = curvatures[k];

          if ( fabs( (K - C).Modulus() ) > visu_KS_limit_min )
          {
            glColor3ub(QrGreen.R, QrGreen.G, QrGreen.B);
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
      }
    glEnd();
  }

  /* =============================
   *  Render U-V curvilinear axes
   * ============================= */

  #pragma region GL lines
  glLineWidth(3);
  glBegin(GL_LINES);

    t_ptr<t_bcurve> iso_V_min = bSurf->Iso_V( bSurf->GetMinParameter_V() );
    t_ptr<t_bcurve> iso_U_min = bSurf->Iso_U( bSurf->GetMinParameter_U() );

    t_xyz P0_iso_V_min, P0_iso_V_min_delta, P0_iso_U_min, P0_iso_U_min_delta;
    iso_V_min->Eval(iso_V_min->GetMinParameter(), P0_iso_V_min);
    iso_U_min->Eval(iso_U_min->GetMinParameter(), P0_iso_U_min);
    iso_V_min->Eval( ( iso_V_min->GetMaxParameter() + iso_V_min->GetMinParameter() )/5, P0_iso_V_min_delta );
    iso_U_min->Eval( ( iso_U_min->GetMaxParameter() + iso_U_min->GetMinParameter() )/5, P0_iso_U_min_delta );

    // OU (fixed V at min)
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f( (GLfloat) P0_iso_V_min.X(), (GLfloat) P0_iso_V_min.Y(), (GLfloat) P0_iso_V_min.Z() );
    glVertex3f( (GLfloat) P0_iso_V_min_delta.X(), (GLfloat) P0_iso_V_min_delta.Y(), (GLfloat) P0_iso_V_min_delta.Z() );

    // OV (fixed U at min)
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f( (GLfloat) P0_iso_U_min.X(), (GLfloat) P0_iso_U_min.Y(), (GLfloat) P0_iso_U_min.Z() );
    glVertex3f( (GLfloat) P0_iso_U_min_delta.X(), (GLfloat) P0_iso_U_min_delta.Y(), (GLfloat) P0_iso_U_min_delta.Z() );

  glEnd();
  glDisable(GL_LINE_SMOOTH);
  #pragma endregion
}

//! Prepares isoline properties for visualization.
//! \param Iso        [in]  target isoline.
//! \param Points     [out] collection of isoline points.
//! \param Curvatures [out] collection of isoline curvature vectors.
void mobius::visu_ActorBSplSurface::fillIsolineProps(const t_ptr<t_bcurve>&             Iso,
                                                     std::vector< std::vector<t_xyz> >& Points,
                                                     std::vector< std::vector<t_xyz> >& Curvatures)
{
  const double pMin = Iso->GetMinParameter();
  const double pMax = Iso->GetMaxParameter();
  const double pStep = (pMax - pMin) / 100.0;
  double p = pMin;

  std::vector<t_xyz> iso;
  std::vector<t_xyz> iso_N;
  bool stop = false;

  while ( !stop )
  {
    if ( p > pMax )
    {
      p = pMax;
      stop = true;
    }

    // Evaluate point on curve
    t_xyz C;
    Iso->Eval(p, C);
    iso.push_back(C);

    // Evaluate curvature on curve
    if ( Iso->GetDegree() >= 2 )
    {
      t_xyz d2C;
      const double K = Iso->K(p);
      Iso->Eval_Dk(p, 2, d2C);
      t_xyz planar_norm = d2C.Normalized();
      planar_norm = planar_norm * K * 10; // Factor for better visualization
      t_xyz C_norm = C + d2C*0.01;/*planar_norm*/;
      iso_N.push_back(C_norm);
    }

    // Go to next step
    p += pStep;
  }

  // Store isoline properties
  Points.push_back(iso);
  Curvatures.push_back(iso_N);
}
