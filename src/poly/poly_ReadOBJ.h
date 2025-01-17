//-----------------------------------------------------------------------------
// Created on: 29 August 2019
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

#ifndef poly_ReadOBJ_HeaderFile
#define poly_ReadOBJ_HeaderFile

// Poly includes
#include <mobius/poly_Mesh.h>

// Core includes
#include <mobius/core_IAlgorithm.h>

namespace mobius {

//! \ingroup MOBIUS_POLY
//!
//! Utility to read OBJ files.
class poly_ReadOBJ : public core_IAlgorithm
{
public:

  //! Ctor.
  //! \param[in] progress progress notifier.
  //! \param[in] plotter  imperative plotter.
  poly_ReadOBJ(core_ProgressEntry progress = nullptr,
               core_PlotterEntry  plotter  = nullptr) : core_IAlgorithm(progress, plotter) {}

public:

  //! \return constructed mesh.
  const t_ptr<t_mesh>& GetResult() const
  {
    return m_mesh;
  }

public:

  //! Reads PLY file.
  //!
  //! \param[in] filename file to read.
  //! \return true in case of success, false -- otherwise.
  mobiusPoly_EXPORT bool
    Perform(const std::string& filename);

protected:

  t_ptr<t_mesh> m_mesh; //!< Mesh data structure.

};

}

#endif
