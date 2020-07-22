//-----------------------------------------------------------------------------
// Created on: 02 September 2014
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
#include <mobius/visu_DataPositionCloud.h>

// core includes
#include <mobius/core_Precision.h>

// STL includes
#include <set>

//! Constructor.
//! \param cloud [in] geometric cloud to visualize.
mobius::visu_DataPositionCloud::visu_DataPositionCloud(const t_ptr<t_pcloud>& cloud)
: visu_DataSet(),
  m_cloud(cloud)
{
}

//! Destructor.
mobius::visu_DataPositionCloud::~visu_DataPositionCloud()
{
}

//! Merges this point cloud with the passed one.
//! \param data [in] position cloud to merge with.
void mobius::visu_DataPositionCloud::Join(const t_ptr<visu_DataSet>& data)
{
  if ( data.IsNull() )
    return;

  t_ptr<visu_DataPositionCloud>
    dataCloud = t_ptr<visu_DataPositionCloud>::DownCast(data);

  if ( m_cloud.IsNull() )
  {
    m_cloud = dataCloud->m_cloud;
    return;
  }

  //---------------------------------------------------------------------------
  // TODO: effective algorithm for merging is really needed. The one below
  //       is of very poor performance
  //---------------------------------------------------------------------------

  // Indices of the new points to be added
  std::vector<int> indices;
  this->excludeRepetitions(m_cloud, dataCloud->m_cloud, indices);

  // Transfer points
  for ( std::vector<int>::const_iterator it = indices.begin(); it != indices.end(); ++it )
    m_cloud->AddPoint( dataCloud->m_cloud->GetPoint(*it) );
}

//! Cleans up data set.
void mobius::visu_DataPositionCloud::Clear()
{
  if ( !m_cloud.IsNull() )
    m_cloud->Clear();
}

//! Traverses over the second cloud and searches for repetitions against
//! the first cloud. The indices of all non-coincident points are stored
//! in the output vector.
//! \param cloud_1         [in]  primary cloud.
//! \param cloud_2         [in]  secondary (donor) cloud.
//! \param indices_to_keep [out] collection of indices to transfer from
//!                              the secondary cloud to the primary one.
void mobius::visu_DataPositionCloud::excludeRepetitions(const t_ptr<t_pcloud>& cloud_1,
                                                        const t_ptr<t_pcloud>& cloud_2,
                                                        std::vector<int>&      indices_to_keep)
{
  // Indices of the new points to be added
  indices_to_keep.clear();

  // Coincidence resolution
  const double prec = core_Precision::Resolution3D();

  // Loop over the passed cloud checking for each point whether it can be
  // added to the original data set or not
  for ( int j = 0; j < cloud_2->GetNumberOfPoints(); ++j )
  {
    const t_xyz& Pj = cloud_2->GetPoint(j);

    // Check next point on coincidence with existing cloud
    bool isOk = true;
    for ( int i = 0; i < cloud_1->GetNumberOfPoints(); ++i )
    {
      const t_xyz& Pi = cloud_1->GetPoint(i);
      const double dist = (Pi - Pj).Modulus();
      if ( dist < prec )
      {
        isOk = false;
        break;
      }
    }

    // Append point to the original cloud
    if ( isOk )
      indices_to_keep.push_back( (int) j );
  }
}
