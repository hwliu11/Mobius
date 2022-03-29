//-----------------------------------------------------------------------------
// Created on: 23 March 2022
//-----------------------------------------------------------------------------
// Copyright (c) 2022-present, Sergey Slyadnev
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

#ifndef poly_BaseDistanceField_HeaderFile
#define poly_BaseDistanceField_HeaderFile

// Poly includes
#include <mobius/poly_RealFunc.h>
#include <mobius/poly_SVO.h>

// Core includes
#include <mobius/core_IProgressNotifier.h>
#include <mobius/core_IPlotter.h>

namespace mobius {

//! \ingroup MOBIUS_POLY
//!
//! Base class for voxel-based distance fields.
class poly_BaseDistanceField : public poly_RealFunc
{
public:

  //! Checks if the passed SVO node is completely inside of the initial shape.
  //! \param[in] pNode SVO node to check.
  //! \return true/false.
  mobiusPoly_EXPORT static bool
    IsIn(poly_SVO* pNode);

  //! Checks if the passed SVO node is completely outside of the initial shape.
  //! \param[in] pNode SVO node to check.
  //! \return true/false.
  mobiusPoly_EXPORT static bool
    IsOut(poly_SVO* pNode);

  //! Checks if the passed SVO node crosses zero isosurface level, i.e., it
  //! captures the boundary of the initial shape.
  //! \param[in] pNode SVO node to check.
  //! \return true/false.
  mobiusPoly_EXPORT static bool
    IsZeroCrossing(poly_SVO* pNode);

public:

  virtual bool
    Build(const double                minCellSize,
          const double                maxCellSize,
          const t_ptr<poly_RealFunc>& func) = 0;

  virtual poly_SVO*
    GetRoot() = 0;

};

}

#endif
