//-----------------------------------------------------------------------------
// Created on: 12 December 2019
//-----------------------------------------------------------------------------
// Copyright (c) 2019-present, Sergey Slyadnev
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

// Poly includes
#include <mobius/poly_DistanceField.h>

// Correction coefficient for error estimation.
#define ERROR_SCALE_COEFF 0.5

//-----------------------------------------------------------------------------

namespace mobius
{
  //! Auxiliary class to split SVO voxels for DDF construction.
  class poly_VoxelSplitTask
  {
  public:

    //! Ctor.
    //! \param[in] pVoxel    SVO node to split.
    //! \param[in] tolerance tolerance value to control the splitting process.
    //!                      The min cell size should never be less than this
    //!                      tolerance.
    //! \param[in] distFunc  distance function.
    poly_VoxelSplitTask(poly_SVO*                       pVoxel,
                        const double                    tolerance,
                        const t_ptr<poly_DistanceFunc>& distFunc)
    //
    : m_pVoxel     (pVoxel),
      m_fTolerance (tolerance)
    {}

  public:

    //! Splits (or not) the working voxel.
    void execute()
    {
      /* =============================================
       *  Get 27 distance probes for the current cell.
       * ============================================= */

      double minDistance = DBL_MAX, maxDistance = -DBL_MAX;

      // Sampled distance values.
      double f[3][3][3] = { { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} },
                            { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} },
                            { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} } };

      // Get distances which are already available at cell nodes.
      for ( int nx = 0; nx < 2; ++nx )
      {
        for ( int ny = 0; ny < 2; ++ny )
        {
          for ( int nz = 0; nz < 2; ++nz )
          {
            const size_t nid = m_pVoxel->GetCornerID(nx, ny, nz);
            const double s   = m_pVoxel->GetScalar(nid);

            minDistance = std::min(std::fabs(s), minDistance);
            maxDistance = std::max(std::fabs(s), maxDistance);

            // Fill only the nodes, i.e., the 0-th and the 2-nd
            // elements of the array. We keep the 1-st elements
            // for sampling in intermediate nodes. 
            f[nx << 1][ny << 1][nz << 1] = s;
          }
        }
      }

      // Get the half-size of the working cell.
      const t_xyz& P0       = m_pVoxel->GetCornerMin();
      const t_xyz& P7       = m_pVoxel->GetCornerMax();
      t_xyz        diagVec  = P7 - P0;
      const double halfSize = 0.5*diagVec.Modulus();

      // Check stop criterion.
      if ( halfSize < m_fTolerance ||  // Max resolution is reached.
           minDistance > halfSize  ||  // No geometry inside.
           maxDistance > 2.*halfSize ) // No geometry inside.
      {
        return; // Halt the splitting process.
      }

      // Add distances at intermediate nodes by evaluating the
      // distance function.
      for ( int nx = 0; nx < 3; ++nx )
      {
        const double samplex = P0 + 0.5*diagVec.X()*nx;
        for ( int ny = 0; ny < 3; ++ny )
        {
          const double sampley = P0 + 0.5*diagVec.Y()*ny;
          for ( int nz = 0; nz < 3; ++nz )
          {
            const double samplez = P0 + 0.5*diagVec.Z()*nz;

            if ( nx == 1 || ny == 1 || nz == 1 ) // Inner point.
            {
              voxelDistances[nx][ny][nz] = m_func->Eval(samplex,
                                                        sampley,
                                                        samplez);
            }
          }
        }
      }

      /* =============================================
       *  Check if the field changes linearly. If not,
       *  the current voxel will be split.
       * ============================================= */

      // Distances at intermediate points to compare with the real distances
      // computed above by the distance function.
      double t[3][3][3] = { { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} },
                            { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} },
                            { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} } };

      // Compute the test distances by averaging the corner ones.
      for ( int i = 0; i < 3; i += 2 )
      {
        for ( int j = 0; j < 3; j += 2 )
        {
          t[1][i][j] = ( f[0][i][j] + f[2][i][j] )*0.5;
          t[i][1][j] = ( f[i][0][j] + f[i][2][j] )*0.5;
          t[i][j][1] = ( f[i][j][0] + f[i][j][2] )*0.5;
        }
      }
      //
      for ( int i = 0; i < 3; i += 2 )
      {
        t[i][1][1] = ( f[i][0][1] + f[i][2][1] )*0.5;
        t[1][i][1] = ( f[0][i][1] + f[2][i][1] )*0.5;
        t[1][1][i] = ( f[0][1][i] + f[2][1][i] )*0.5;
      }
      //
      t[1][1][1] = ( f[0][1][1] + f[2][1][1] )*0.5;

      // If the average distance is significantly greater than the fairly
      // computed distance, then the linear approximation is not sufficiently
      // precise. In such situation, the voxel should be split.
      bool toSplit = false;
      for ( int nx = 0; nx < 3; ++nx )
      {
        for ( int ny = 0; ny < 3; ++ny )
        {
          for (int nz = 0; nz < 3; ++nz )
          {
            if ( nx == 1 || ny == 1 || nz == 1 ) // Any inner point.
            {
              toSplit |= std::fabs( f[nx][nz][aZ] - t[aX][aY][aZ] ) > (m_fTolerance*ERROR_SCALE_COEFF);
            }
          }
        }
      }

      if ( !toSplit )
        return;

      /* ========================================
       *  Split the current cell into 8 children.
       * ======================================== */

      if ( m_pVoxel->IsLeaf() )
      {
        m_pVoxel->Split(); // This only creates pointers.

        // Allocate voxels.
        for ( size_t subID = 0; subID < 8; ++subID )
          m_pVoxel->SetChild(subID, new poly_SVO);
      }

      for ( size_t subID = 0; subID < 8; ++subID )
      {
        poly_SVO* pChild = m_pVoxel->GetChild(subID);

        size_t nx, ny, nz;
        poly_SVO::GetCornerLocation(subID, nx, ny, nz);

        // P0 of the new octant.
        const t_xyz childP0( P0.x() + 0.5*diagVec.X()*nx,
                             P0.y() + 0.5*diagVec.Y()*ny,
                             P0.z() + 0.5*diagVec.Z()*nz );

        // P7 of the new octant.
        const t_xyz childP7( P7.x() - 0.5*diagVec.x()*(1 - nx),
                             P7.y() - 0.5*diagVec.y()*(1 - ny),
                             P7.z() - 0.5*diagVec.z()*(1 - nz) );

        // Store corners.
        pChild->P0 = childP0;
        pChild->P7 = childP7;

        // Store scalars.
        for ( int cnx = 0; cnx < 3; ++cnx )
        {
          for ( int cny = 0; cny < 3; ++cny )
          {
            for (int cnz = 0; cnz < 3; ++cnz )
            {
              const double dist = f[cnx + nx][cny + ny][cnz + nz];

              pChild->SetDistance(poly_SVO::GetCornerID(cnx, cny, cnz), dist);
            }
          }
        }
      }

    }

  private:

    poly_SVO*                m_pVoxel;     //!< Working SVO node.
    double                   m_fTolerance; //!< Tolerance to control the SVO fineness.
    t_ptr<poly_DistanceFunc> m_func;       //!< Distance function.

  };
}

//-----------------------------------------------------------------------------

mobius::poly_DistanceField::poly_DistanceField(core_ProgressEntry progress,
                                               core_PlotterEntry  plotter)
: core_OBJECT (),
  m_pRoot     (nullptr),
  m_progress  (progress),
  m_plotter   (plotter)
{}

//-----------------------------------------------------------------------------

mobius::poly_DistanceField::~poly_DistanceField()
{
  if ( m_pRoot )
    delete m_pRoot; // Call SVO dtor which releases the octree recursively.
}

//-----------------------------------------------------------------------------

bool mobius::poly_DistanceField::Build(const double                    poly_NotUsed(resolution),
                                       const t_ptr<poly_DistanceFunc>& func)
{
  if ( func.IsNull() )
  {
    m_progress.SendLogMessage(MobiusErr(Normal) << "Null function.");
    return false;
  }

  const t_xyz& cornerMin = func->GetDomainMin();
  const t_xyz& cornerMax = func->GetDomainMax();

  /* ==================
   *  Create root node.
   * ================== */

  if ( m_pRoot )
    delete m_pRoot; // Call SVO dtor which releases the octree recursively.

  // Create root SVO node and initialize it with the distance scalars.
  m_pRoot = new poly_SVO(cornerMin, cornerMax);
  //
  for ( size_t nx = 0; nx < 1; ++nx )
  {
    const double x = ( (nx == 0) ? cornerMin.X() : cornerMax.X() );
    for ( size_t ny = 0; ny < 1; ++ny )
    {
      const double y = ( (ny == 0) ? cornerMin.Y() : cornerMax.Y() );
      for ( size_t nz = 0; nz < 1; ++nz )
      {
        const double z = ( (nz == 0) ? cornerMin.Z() : cornerMax.Z() );

        // Evaluate distance.
        const double f = func->Eval(x, y, z);

        // Set scalar.
        m_pRoot->SetScalar(poly_SVO::GetCornerID(nx, ny, nz), f);
      }
    }
  }

  /* ==================
   *  Create hierarchy.
   * ================== */

  // TODO: NYI

  return false;
}
