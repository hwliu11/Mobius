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

#ifndef visu_VarRepo_HeaderFile
#define visu_VarRepo_HeaderFile

// visu includes
#include <mobius/visu.h>

// core includes
#include <mobius/core_Ptr.h>

// STD includes
#include <map>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Transient repository of variables against their names. Each variable
//! has to be a shared object.
class visu_VarRepo
{
public:

  //! Type shortcut for the pair used in repository map.
  typedef std::pair< std::string, t_ptr<core_OBJECT> > TRepoPair;

  //! Type shortcut for repository map.
  typedef std::map< std::string, t_ptr<core_OBJECT> > TRepoMap;

public:

  inline visu_VarRepo() {}
  inline virtual ~visu_VarRepo() {}

public:

  mobiusVisu_EXPORT void
    RegisterVariable(const std::string& Name,
                     const t_ptr<core_OBJECT>& Var);

  mobiusVisu_EXPORT t_ptr<core_OBJECT>
    FindVariable(const std::string& Name);

  mobiusVisu_EXPORT const TRepoMap&
    Variables();

private:

  visu_VarRepo(const visu_VarRepo&) {}
  void operator=(const visu_VarRepo&) {}

private:

  //! Map of variables against their names.
  TRepoMap __repo;

};

}

#endif
