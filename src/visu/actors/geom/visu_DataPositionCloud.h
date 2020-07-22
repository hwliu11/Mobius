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

#ifndef visu_DataPositionCloud_HeaderFile
#define visu_DataPositionCloud_HeaderFile

// visu includes
#include <mobius/visu_DataSet.h>

// geom includes
#include <mobius/geom_PositionCloud.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Position cloud data.
class visu_DataPositionCloud : public visu_DataSet
{
public:

  mobiusVisu_EXPORT
    visu_DataPositionCloud(const t_ptr<t_pcloud>& cloud = NULL);

  mobiusVisu_EXPORT virtual
    ~visu_DataPositionCloud();

public:

  //! Sets geometric point cloud.
  //! \param cloud [in] geometric position cloud to set for visualization.
  void SetCloud(const t_ptr<t_pcloud>& cloud)
  {
    m_cloud = cloud;
  }

  //! Returns geometric point cloud.
  //! \return geometric position cloud.
  const t_ptr<t_pcloud>& GetCloud() const
  {
    return m_cloud;
  }

  //! Adds the passed point to the data set.
  void AddPoint(const t_xyz& point)
  {
    if ( m_cloud.IsNull() )
      m_cloud = new t_pcloud();

    m_cloud->AddPoint(point);
  }

public:

  mobiusVisu_EXPORT virtual void
    Join(const t_ptr<visu_DataSet>& data);

  mobiusVisu_EXPORT virtual void
    Clear();

private:

  void excludeRepetitions(const t_ptr<t_pcloud>& cloud_1,
                          const t_ptr<t_pcloud>& cloud_2,
                          std::vector<int>&    indices_to_keep);

private:

  //! Geometric point cloud to visualize.
  t_ptr<t_pcloud> m_cloud;

};

}

#endif
