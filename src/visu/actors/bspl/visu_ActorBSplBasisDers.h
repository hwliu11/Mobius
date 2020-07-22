//-----------------------------------------------------------------------------
// Created on: 15 February 2014
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

#ifndef visu_ActorBSplBasisDers_HeaderFile
#define visu_ActorBSplBasisDers_HeaderFile

// visu includes
#include <mobius/visu_ActorInsensitive.h>

// core includes
#include <mobius/core_HeapAlloc.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Class representing OpenGL Actor dedicated to visualization of derivatives
//! of B-spline basis functions.
class visu_ActorBSplBasisDers : public visu_ActorInsensitive
{
public:

  mobiusVisu_EXPORT
    visu_ActorBSplBasisDers(const std::vector<double>& U,
                              const int                  deg,
                              const int                  k);

  mobiusVisu_EXPORT virtual
    ~visu_ActorBSplBasisDers();

public:

  mobiusVisu_EXPORT virtual void
    GetBounds(double& xMin, double& xMax,
              double& yMin, double& yMax,
              double& zMin, double& zMax) const;

  mobiusVisu_EXPORT virtual void
    GL_Draw();

private:

  std::vector<double>        m_U;       //!< Knots.
  int                        m_iDeg;    //!< Degree of B-spline basis functions.
  int                        m_iOrder;  //!< Order of derivatives to render.
  double**                   m_pF;      //!< Evaluated functions/derivatives to render.
  core_HeapAlloc2D<double> m_alloc2D; //!< Heap allocator.

};

}

#endif
