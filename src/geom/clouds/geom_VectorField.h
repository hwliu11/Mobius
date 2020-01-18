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

#ifndef geom_VectorField_HeaderFile
#define geom_VectorField_HeaderFile

// Geometry includes
#include <mobius/geom_PositionCloud.h>

// STL includes
#include <map>

namespace mobius {

//! \ingroup MOBIUS_GEOM
//!
//! Represents position point cloud with associated vectors. Ideally, one
//! should have a vector associated with each point in the cloud. However,
//! if you do not have any, this container assumes that zero vector is
//! there. Such zero vectors are not stored in the Vector Field objects.
class geom_VectorField : public core_OBJECT
{
// Construction & destruction:
public:

  mobiusGeom_EXPORT
    geom_VectorField(const t_ptr<t_pcloud>& cloud);

  mobiusGeom_EXPORT virtual
    ~geom_VectorField();

public:

  mobiusGeom_EXPORT virtual void
    Clear();

public:

  mobiusGeom_EXPORT t_ptr<geom_VectorField>
    Copy() const;

  mobiusGeom_EXPORT const t_ptr<t_pcloud>&
    GetCloud() const;

  mobiusGeom_EXPORT t_ptr<t_pcloud>&
    ChangeCloud();

  mobiusGeom_EXPORT void
    AddVector(const size_t pnt_index, const t_xyz& vector);

  mobiusGeom_EXPORT t_xyz
    GetVector(const size_t pnt_index) const;

  mobiusGeom_EXPORT bool
    HasVectors() const;

private:

  //! Positions for vectors.
  t_ptr<t_pcloud> m_cloud;

  //! Collection of vectors associated with points.
  std::map<size_t, t_xyz> m_vectors;

};

//! Handy shortcut.
typedef geom_VectorField t_vector_field;

}

#endif
