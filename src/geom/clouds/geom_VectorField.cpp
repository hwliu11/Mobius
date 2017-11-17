//-----------------------------------------------------------------------------
// Created on: 25 February 2015
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

// Own include
#include <mobius/geom_VectorField.h>

//! Constructs degenerated vector field (all vectors are nulls) under the
//! given position point cloud.
//! \param cloud [in] points.
mobius::geom_VectorField::geom_VectorField(const Ptr<pcloud>& cloud)
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
mobius::Ptr<mobius::geom_VectorField> mobius::geom_VectorField::Copy() const
{
  Ptr<geom_VectorField> res = new geom_VectorField(m_cloud);
  res->m_cloud = new pcloud( m_cloud->Points() );
  res->m_vectors = m_vectors;
  return res;
}

//! Returns the associated point cloud.
//! \return point cloud.
const mobius::Ptr<mobius::pcloud>& mobius::geom_VectorField::Cloud() const
{
  return m_cloud;
}

//! Returns the associated point cloud for modification.
//! \return point cloud.
mobius::Ptr<mobius::pcloud>& mobius::geom_VectorField::ChangeCloud()
{
  return m_cloud;
}

//! Associates the passed vector with the point having the given index.
//! \param pnt_index [in] 0-based index of a point.
//! \param vector [in] vector to bind to a point.
void mobius::geom_VectorField::AddVector(const size_t pnt_index,
                                         const xyz&   vector)
{
  m_vectors.insert( std::pair<size_t, xyz>(pnt_index, vector) );
}

//! Returns vector associated with the given point index.
//! \param pnt_index [in] 0-based index of a point.
//! \return requested vector or null vector if nothing was found for the
//!         given point.
mobius::xyz mobius::geom_VectorField::Vector(const size_t pnt_index) const
{
  std::map<size_t, xyz>::const_iterator cit = m_vectors.find(pnt_index);
  if ( cit == m_vectors.cend() )
    return xyz();

  return cit->second;
}

//! Returns true if there are some vectors in the field.
//! \return true/false.
bool mobius::geom_VectorField::HasVectors() const
{
  return m_vectors.size() > 0;
}
