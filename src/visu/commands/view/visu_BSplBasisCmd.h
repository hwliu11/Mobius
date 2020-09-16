//-----------------------------------------------------------------------------
// Created on: 24 February 2015
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

#ifndef visu_BSplBasisCmd_HeaderFile
#define visu_BSplBasisCmd_HeaderFile

// visu includes
#include <mobius/visu_ActorBSplBasis.h>
#include <mobius/visu_ViewCmd.h>

// core includes
#include <mobius/core_HeapAlloc.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Draws B-spline basis functions.
class visu_BSplBasisCmd : public visu_ViewCmd
{
public:

  //! Constructor.
  //! \param CmdRepo [in] command repo.
  //! \param Picker [in] instance of Picker.
  visu_BSplBasisCmd(const t_ptr<visu_CommandRepo>& CmdRepo,
                      const t_ptr<visu_Picker>& Picker)
  : visu_ViewCmd(CmdRepo, Picker) {}

  //! Destructor.
  virtual ~visu_BSplBasisCmd() {}

public:

  //! Returns human-readable name of the command.
  //! \return name.
  inline virtual std::string Name() const
  {
    return "B-spline basis functions";
  }

public:

  mobiusVisu_EXPORT virtual bool
    Execute();

};

}

#endif