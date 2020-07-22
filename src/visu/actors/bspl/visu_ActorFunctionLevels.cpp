//-----------------------------------------------------------------------------
// Created on: 12 October 2015
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
#include <mobius/visu_ActorFunctionLevels.h>

//! Constructor.
//! \param func         [in] function to visualize.
//! \param stratums     [in] function level stratums.
//! \param num_stratums [in] number of level stratums.
//! \param point_size   [in] point size.
mobius::visu_ActorFunctionLevels::visu_ActorFunctionLevels(const t_ptr<core_TwovariateFunc>& func,
                                                           const double*                     stratums,
                                                           const int                         num_stratums,
                                                           const GLfloat                     point_size)
: visu_ActorInsensitive(),
  m_func       (func),
  m_iStratums  (num_stratums),
  m_fPointSize (point_size)
{
  // Copy stratums
  m_pStratums = new double[num_stratums];
  memcpy_s( m_pStratums, num_stratums*sizeof(double), stratums, num_stratums*sizeof(double) );

  // Evaluate target function
  for ( int s = 0; s < num_stratums; ++s )
  {
    const double stratum_h = m_pStratums[s];
    const double stratum_l = ( (s == num_stratums) ? 0.0 : m_pStratums[s + 1] );

    // Create new point cloud
    t_ptr<t_pcloud> cloud = new t_pcloud;

    // Iterate over the space of arguments
    const double XY_LIM      = 1.0; // Shift [m] in all directions
    const double XY_LIM_Step = 0.01;
    double       tx          = 0;
    while ( tx <= XY_LIM ) // Iteration by OX
    {
      t_xyz C;

      double ty = 0;
      while ( ty <= XY_LIM ) // Iteration by OY
      {
        t_xyz G;

        // Calculate distance
        const double rho = m_func->Eval(tx, ty);

        if ( rho < stratum_h && rho > stratum_l )
        {
          t_xyz P(tx, ty, 0.0);
          cloud->AddPoint(P);
        }

        // Increment step over OY
        ty += XY_LIM_Step;
      }

      // Increment step over OX
      tx += XY_LIM_Step;
    }

    GLfloat* vertices = new GLfloat[cloud->GetNumberOfPoints()*3];
    int idx = 0;
    for ( int p = 0; p < cloud->GetNumberOfPoints(); ++p )
    {
      const t_xyz& point = cloud->GetPoint(p);
      vertices[idx++] = (GLfloat) point.X();
      vertices[idx++] = (GLfloat) point.Y();
      vertices[idx++] = (GLfloat) point.Z();
    }

    t_level_set level_set;
    visu_ColorSelector::ColorByIndex_d(s, level_set.color);
    level_set.pts     = vertices;
    level_set.num_pts = (unsigned) cloud->GetNumberOfPoints();

    // Add cloud to the collection of clouds
    m_levels.push_back(level_set);
  }
}

//! Destructor.
mobius::visu_ActorFunctionLevels::~visu_ActorFunctionLevels()
{
  for ( size_t k = 0; k < m_levels.size(); ++k )
  {
    delete[] m_levels[k].pts;
  }
}

//! Calculates boundary box for this Actor.
//! \param xMin [out] min X.
//! \param xMax [out] max X.
//! \param yMin [out] min Y.
//! \param yMax [out] max Y.
//! \param zMin [out] min Z.
//! \param zMax [out] max Z.
void mobius::visu_ActorFunctionLevels::GetBounds(double& xMin, double& xMax,
                                                 double& yMin, double& yMax,
                                                 double& zMin, double& zMax) const
{
  xMin = 0.0;
  xMax = 1.0;
  yMin = 0.0;
  yMax = 1.0;
  zMin = zMax = 0.0;
}

//! Draws function.
void mobius::visu_ActorFunctionLevels::GL_Draw()
{
  glEnable(GL_POINT_SMOOTH);
  glPointSize(m_fPointSize);

  for ( size_t cidx = 0; cidx < m_levels.size(); ++cidx )
  {
    const t_level_set& level = m_levels[cidx];

    glColor3ub(level.color.R, level.color.G, level.color.B);
    glEnableClientState(GL_VERTEX_ARRAY);
      glVertexPointer(3, GL_FLOAT, 0, level.pts);
      glDrawArrays(GL_POINTS, 0, (int) level.num_pts);
    glDisableClientState(GL_VERTEX_ARRAY);
  }

  glPointSize(1.0f);
  glDisable(GL_POINT_SMOOTH);
}
