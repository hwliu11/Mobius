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
    geom_VectorField(const ptr<pcloud>& cloud);

  mobiusGeom_EXPORT virtual
    ~geom_VectorField();

public:

  mobiusGeom_EXPORT virtual void
    Clear();

public:

  mobiusGeom_EXPORT ptr<geom_VectorField>
    Copy() const;

  mobiusGeom_EXPORT const ptr<pcloud>&
    Cloud() const;

  mobiusGeom_EXPORT ptr<pcloud>&
    ChangeCloud();

  mobiusGeom_EXPORT void
    AddVector(const size_t pnt_index, const xyz& vector);

  mobiusGeom_EXPORT xyz
    Vector(const size_t pnt_index) const;

  mobiusGeom_EXPORT bool
    HasVectors() const;

private:

  //! Positions for vectors.
  ptr<pcloud> m_cloud;

  //! Collection of vectors associated with points.
  std::map<size_t, xyz> m_vectors;

};

//! Handy shortcut.
typedef geom_VectorField vector_field;

};

#endif
