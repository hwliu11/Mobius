//-----------------------------------------------------------------------------
// Created on: 17 December 2014
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
#include <mobius/visu_VarRepo.h>

//! Registers variable in repository.
//! \param Name [in] name of the variable.
//! \param Var [in] variable to register.
void mobius::visu_VarRepo::RegisterVariable(const std::string&        Name,
                                            const t_ptr<core_OBJECT>& Var)
{
  TRepoMap::const_iterator cit = __repo.find(Name);
  if ( cit != __repo.end() )
    __repo.erase(cit);

  __repo.insert( TRepoPair(Name, Var) );
}

//! Attempts to find variable in repository by its name.
//! \param Name [in] name of the variable to search for.
//! \return pointer to the variable if anything has been found. If not, the
//!         resulting pointer is null.
mobius::t_ptr<mobius::core_OBJECT>
  mobius::visu_VarRepo::FindVariable(const std::string& Name)
{
  TRepoMap::const_iterator cit = __repo.find(Name);
  if ( cit == __repo.end() )
    return NULL;

  return cit->second;
}

//! Returns the entire collection of variables.
//! \return map of variables.
const mobius::visu_VarRepo::TRepoMap&
  mobius::visu_VarRepo::Variables()
{
  return __repo;
}
