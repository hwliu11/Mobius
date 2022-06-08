//-----------------------------------------------------------------------------
// Created on: 20 September 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2018-present, Sergey Slyadnev
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

#ifndef cascade_Triangulation_HeaderFile
#define cascade_Triangulation_HeaderFile

// Cascade includes
#include <mobius/cascade.h>

// Core includes
#include <mobius/core_Ptr.h>

// Poly includes
#include <mobius/poly_Mesh.h>

// OCCT includes
#include <Poly_Triangulation.hxx>

namespace mobius {

//! \ingroup MOBIUS_CASCADE
//!
//! Bridge for conversions between Mobius and OCCT triangulations.
class cascade_Triangulation
{
public:

  mobiusCascade_EXPORT
    cascade_Triangulation(const t_ptr<poly_Mesh>& mobiusMesh);

  mobiusCascade_EXPORT
    cascade_Triangulation(const Handle(Poly_Triangulation)& occtMesh);

  mobiusCascade_EXPORT
    ~cascade_Triangulation();

public:

  mobiusCascade_EXPORT void
    DirectConvert();

public:

  mobiusCascade_EXPORT const t_ptr<poly_Mesh>&
    GetMobiusTriangulation() const;

  mobiusCascade_EXPORT const Handle(Poly_Triangulation)&
    GetOpenCascadeTriangulation() const;

  mobiusCascade_EXPORT bool
    IsDone() const;

protected:

  mobiusCascade_EXPORT void
    convertToOpenCascade();

  mobiusCascade_EXPORT void
    convertToMobius();

private:

  //! Mobius data structure.
  t_ptr<poly_Mesh> m_mobiusMesh;

  //! OCCT data structure.
  Handle(Poly_Triangulation) m_occtMesh;

  //! Indicates whether conversion is done or not.
  bool m_bIsDone;

};

}

#endif
