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

#ifndef visu_CommandRepo_HeaderFile
#define visu_CommandRepo_HeaderFile

// visu includes
#include <mobius/visu_VarRepo.h>

// core includes
#include <mobius/core_Ptr.h>

// STD includes
#include <map>

namespace mobius {

class visu_BaseCmd;

//! \ingroup MOBIUS_VISU
//!
//! Transient repository of commands against their names.
class visu_CommandRepo : public core_OBJECT
{
public:

  visu_CommandRepo() : core_OBJECT() {}
  ~visu_CommandRepo() {}

public:

  //! Type shortcut for the pair used in repository map.
  typedef std::pair< std::string, t_ptr<visu_BaseCmd> > TRepoPair;

  //! Type shortcut for repository map.
  typedef std::map< std::string, t_ptr<visu_BaseCmd> > TRepoMap;

public:

  mobiusVisu_EXPORT void
    RegisterCommand(const std::string& Name,
                    const t_ptr<visu_BaseCmd>& Command);

  mobiusVisu_EXPORT t_ptr<visu_BaseCmd>
    FindCommand(const std::string& Name);

  mobiusVisu_EXPORT const TRepoMap&
    Commands();

public:

  //! Accessor to the repository of variables.
  //! \return variables repo.
  inline visu_VarRepo& Vars()
  {
    return __varRepo;
  }

private:

  visu_CommandRepo(const visu_CommandRepo&) {}
  void operator=(const visu_CommandRepo&) {}

private:

  //! Map of commands against their names.
  TRepoMap __repo;

  //! Variables associated with this repository of commands and
  //! accessible in each stored command so.
  visu_VarRepo __varRepo;

};

}

#endif
