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

#ifndef visu_BaseCmd_HeaderFile
#define visu_BaseCmd_HeaderFile

// visu includes
#include <mobius/visu_CommandRepo.h>

// core includes
#include <mobius/core_Ptr.h>

namespace mobius {

//! \ingroup MOBIUS_VISU
//!
//! Base class for commands.
class visu_BaseCmd : public core_OBJECT
{
public:

  mobiusVisu_EXPORT
    visu_BaseCmd(const t_ptr<visu_CommandRepo>& cmd_repo);

  mobiusVisu_EXPORT virtual
    ~visu_BaseCmd();

public:

  virtual std::string
    Name() const = 0;

  virtual bool
    Execute() = 0;

public:

  //! Returns the number of arguments.
  //! \return number of arguments.
  inline size_t Argc() const
  {
    return m_args.size();
  }

  //! Alias for Arguments().
  //! \return collection of arguments.
  inline const std::vector<std::string>& Argv() const
  {
    return this->Arguments();
  }

  //! Returns the collection of actual arguments.
  //! \return collection of arguments.
  inline const std::vector<std::string>& Arguments() const
  {
    return m_args;
  }

  //! Accessor for an argument by index.
  //! \param idx [in] 0-based index.
  //! \return argument.
  inline std::string Arg(const size_t idx) const
  {
    if ( idx >= m_args.size() )
      return "";

    return m_args[idx];
  }

  //! Accessor for an argument by index. Here we presume that the argument
  //! is of the given type.
  //! \param idx [in] 0-based index.
  //! \param default_value [in] default value.
  //! \return argument.
  template<typename T>
  inline T Arg(const size_t idx,
               const T&     default_value) const
  {
    std::string arg = this->Arg(idx);
    if ( !arg.size() )
      return default_value;

    return core::str::to_number<T>(arg);
  }

  //! Specialization for standard string.
  inline std::string Arg(const size_t       idx,
                         const std::string& default_value) const
  {
    std::string arg = this->Arg(idx);
    if ( !arg.size() )
      return default_value;

    return arg;
  }

  //! Sets arguments.
  //! \param args [in] arguments to set.
  inline void SetArguments(const std::vector<std::string>& args)
  {
    m_args = args;
  }

  //! Accessor for the repository of commands.
  //! \return command repo.
  inline const t_ptr<visu_CommandRepo>& CmdRepo() const
  {
    return m_cmdRepo;
  }

protected:

  //! Collection of arguments passed with the command.
  std::vector<std::string> m_args;

  //! Command repo.
  t_ptr<visu_CommandRepo> m_cmdRepo;

};

}

#endif
