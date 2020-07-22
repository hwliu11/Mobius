//-----------------------------------------------------------------------------
// Created on: 30 April 2014
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
#include <mobius/visu_CommandRepo.h>

// visu includes
#include <mobius/visu_BaseCmd.h>

//! Registers command in repository.
//! \param Name [in] name of the command.
//! \param Command [in] command to register.
void mobius::visu_CommandRepo::RegisterCommand(const std::string& Name,
                                               const t_ptr<visu_BaseCmd>& Command)
{
  std::vector<std::string> commands;
  core::str::split(Name, " ", commands);

  for ( size_t i = 0; i < commands.size(); ++i )
    __repo.insert( TRepoPair(commands[i], Command) );
}

//! Attempts to find command in repository by the command's name.
//! \param Name [in] name of the command to search for.
//! \return pointer to the command if anything has been found. If not, the
//!         resulting pointer is null.
mobius::t_ptr<mobius::visu_BaseCmd>
  mobius::visu_CommandRepo::FindCommand(const std::string& Name)
{
  TRepoMap::const_iterator cit = __repo.find(Name);
  if ( cit == __repo.end() )
    return NULL;

  return cit->second;
}

//! Returns the entire collection of commands.
//! \return map of commands.
const mobius::visu_CommandRepo::TRepoMap&
  mobius::visu_CommandRepo::Commands()
{
  return __repo;
}
