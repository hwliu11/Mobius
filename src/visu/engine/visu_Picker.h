//-----------------------------------------------------------------------------
// Created on: 21 May 2014
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

#ifndef visu_Picker_HeaderFile
#define visu_Picker_HeaderFile

// visu includes
#include <mobius/visu_Scene.h>
#include <mobius/visu_Utils.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Picker for objects. Implements very simple approach of picking:
//! intersection with a ray shot from camera's position.
class visu_Picker : public core_OBJECT
{
public:

  //! Picking mode.
  enum Mode
  {
    Mode_Single, //!< Single object picking.
    Mode_Join    //!< Joint object picking.
  };

public:

  mobiusVisu_EXPORT
    visu_Picker(const t_ptr<visu_Scene>& Scene);

public:

  mobiusVisu_EXPORT void
    Pick(const int mouseX,
         const int mouseY);

  mobiusVisu_EXPORT void
    SetMode(const Mode mode = Mode_Single);

  mobiusVisu_EXPORT Mode
    GetMode() const;

  mobiusVisu_EXPORT const t_ptr<visu_Scene>&
    Scene() const;

private:

  //! Scene.
  t_ptr<visu_Scene> m_scene;

  //! Picking mode.
  Mode m_mode;

};

}

#endif
