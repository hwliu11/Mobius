//-----------------------------------------------------------------------------
// Created on: 25 February 2015
//-----------------------------------------------------------------------------
// Copyright (c) 2017, Sergey Slyadnev
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
#include <mobius/geom_VectorField.h>

//! Constructs degenerated vector field (all vectors are nulls) under the
//! given position point cloud.
//! \param cloud [in] points.
mobius::geom_VectorField::geom_VectorField(const t_ptr<t_pcloud>& cloud)
: core_OBJECT(),
  m_cloud(cloud)
{}

//! Destructor.
mobius::geom_VectorField::~geom_VectorField()
{}

//! Cleans up the vector field. The associated point cloud remains untouched.
void mobius::geom_VectorField::Clear()
{
  m_vectors.clear();
}

//! Prepares a copy of this vector field.
//! \return copy.
mobius::t_ptr<mobius::geom_VectorField> mobius::geom_VectorField::Copy() const
{
  t_ptr<geom_VectorField> res = new geom_VectorField(m_cloud);
  res->m_cloud = new t_pcloud( m_cloud->GetPoints() );
  res->m_vectors = m_vectors;
  return res;
}

//! Returns the associated point cloud.
//! \return point cloud.
const mobius::t_ptr<mobius::t_pcloud>& mobius::geom_VectorField::GetCloud() const
{
  return m_cloud;
}

//! Returns the associated point cloud for modification.
//! \return point cloud.
mobius::t_ptr<mobius::t_pcloud>& mobius::geom_VectorField::ChangeCloud()
{
  return m_cloud;
}

//! Associates the passed vector with the point having the given index.
//! \param pnt_index [in] 0-based index of a point.
//! \param vector [in] vector to bind to a point.
void mobius::geom_VectorField::AddVector(const size_t pnt_index,
                                         const t_xyz& vector)
{
  m_vectors.insert( std::pair<size_t, t_xyz>(pnt_index, vector) );
}

//! Returns vector associated with the given point index.
//! \param pnt_index [in] 0-based index of a point.
//! \return requested vector or null vector if nothing was found for the
//!         given point.
mobius::t_xyz mobius::geom_VectorField::GetVector(const size_t pnt_index) const
{
  std::map<size_t, t_xyz>::const_iterator cit = m_vectors.find(pnt_index);
  if ( cit == m_vectors.cend() )
    return t_xyz();

  return cit->second;
}

//! Returns true if there are some vectors in the field.
//! \return true/false.
bool mobius::geom_VectorField::HasVectors() const
{
  return m_vectors.size() > 0;
}
