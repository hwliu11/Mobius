//-----------------------------------------------------------------------------
// Created on: 24 July 2020
//-----------------------------------------------------------------------------
// Copyright (c) 2020-present, Sergey Slyadnev
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

#ifndef visu_LoadSTLCmd_HeaderFile
#define visu_LoadSTLCmd_HeaderFile

// visu includes
#include <mobius/visu_ViewCmd.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Loads mesh from an STL file.
class visu_LoadSTLCmd : public visu_ViewCmd
{
public:

  //! Ctor.
  //! \param[in] cmds   commands repo.
  //! \param[in] picker interactive picker.
  visu_LoadSTLCmd(const t_ptr<visu_CommandRepo>& cmds,
                  const t_ptr<visu_Picker>&      picker)
  : visu_ViewCmd(cmds, picker) {}

  //! Dtor.
  virtual ~visu_LoadSTLCmd() = default;

public:

  //! Returns human-readable name of the command.
  //! \return name.
  virtual std::string Name() const
  {
    return "Load STL";
  }

public:

  //! Runs the command.
  //! \return true in case of success, false -- otherwise.
  mobiusVisu_EXPORT virtual bool
    Execute();

};

}

#endif
