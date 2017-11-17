//-----------------------------------------------------------------------------
// Created on: 25 February 2015
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef geom_VectorField_HeaderFile
#define geom_VectorField_HeaderFile

// Geometry includes
#include <mobius/geom_PositionCloud.h>

// STL includes
#include <map>

namespace mobius {

//! Represents position point cloud with associated vectors. Ideally, one
//! should have a vector associated with each point in the cloud. However,
//! if you don't have some, this container assumes that zero vector is
//! there. Of course, zero vectors are not stored in the Vector Field
//! objects.
class geom_VectorField : public core_OBJECT
{
// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_VectorField(const Ptr<pcloud>& cloud);

  mobiusGeom_EXPORT virtual
    ~geom_VectorField();

public:

  mobiusGeom_EXPORT virtual void
    Clear();

public:

  mobiusGeom_EXPORT Ptr<geom_VectorField>
    Copy() const;

  mobiusGeom_EXPORT const Ptr<pcloud>&
    Cloud() const;

  mobiusGeom_EXPORT Ptr<pcloud>&
    ChangeCloud();

  mobiusGeom_EXPORT void
    AddVector(const size_t pnt_index, const xyz& vector);

  mobiusGeom_EXPORT xyz
    Vector(const size_t pnt_index) const;

  mobiusGeom_EXPORT bool
    HasVectors() const;

private:

  //! Positions for vectors.
  Ptr<pcloud> m_cloud;

  //! Collection of vectors associated with points.
  std::map<size_t, xyz> m_vectors;

};

//! Handy shortcut.
typedef geom_VectorField vector_field;

};

#endif
