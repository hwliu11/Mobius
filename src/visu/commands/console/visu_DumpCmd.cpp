//-----------------------------------------------------------------------------
// Created on: 25 December 2014
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

// Own include
#include <mobius/visu_DumpCmd.h>

// visu includes
#include <mobius/visu_CommandRepo.h>

//! Constructor.
//! \param cmd_repo [in] command repo.
mobius::visu_DumpCmd::visu_DumpCmd(const t_ptr<visu_CommandRepo>& cmd_repo) : visu_ConsoleCmd(cmd_repo)
{
}

//! Destructor.
mobius::visu_DumpCmd::~visu_DumpCmd()
{
}

//! Executes command.
//! \return true in case of success, false -- otherwise.
bool mobius::visu_DumpCmd::Execute()
{
  std::cout << this->Name() << std::endl;
  if ( this->Argc() != 1 )
  {
    std::cout << "Error: you have to specify variable name" << std::endl;
    return false;
  }

  std::string varName = this->Arg<std::string>(0, "");
  t_ptr<core_OBJECT> var = this->CmdRepo()->Vars().FindVariable(varName);
  if ( var.IsNull() )
  {
    std::cout << "Error: no such variable" << std::endl;
    return false;
  }

  // Dump
  std::stringstream stream;
  var->Dump(&stream);
  std::cout << stream.str();

  return true;
}
