//-----------------------------------------------------------------------------
// Created on: July 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2018-present, OPEN CASCADE SAS
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
//    * Neither the name of OPEN CASCADE SAS nor the
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
//
// Web: http://dev.opencascade.org
//-----------------------------------------------------------------------------

#ifndef core_Vars_HeaderFile
#define core_Vars_HeaderFile

// Core includes
#include <mobius/core_Ptr.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Base class for variables which are the shared objects having some
//! named values.
class core_VarBase : public core_OBJECT
{
public:

  //! Variable name.
  std::string Name;

protected:

  //! Default ctor.
  core_VarBase() : core_OBJECT()
  {}

};

//-----------------------------------------------------------------------------
// Integer
//-----------------------------------------------------------------------------

//! \ingroup MOBIUS_CORE
//!
//! Integer variable.
class core_VarInt : public core_VarBase
{
public:

  //! Factory method ensuring allocation of Variable instance in a heap.
  //! \param[in] name  variable name.
  //! \param[in] value variable value.
  //! \return variable instance.
  static t_ptr<core_VarInt>
    Instance(const std::string& name,
             const int          value)
  {
    return new core_VarInt(name, value);
  }

public:

  //! Variable value.
  int Value;

public:

  //! Default constructor.
  core_VarInt() : core_VarBase()
  {}

  //! Constructor accepting variable value as an argument. The variable will
  //! have no name (empty name member).
  //! \param[in] value variable value.
  core_VarInt(const int value) : core_VarBase()
  {
    Value = value;
  }

  //! Constructor accepting the name and the value of the variable.
  //! \param[in] name  variable name.
  //! \param[in] value variable value.
  core_VarInt(const std::string& name,
              const int          value) : core_VarBase()
  {
    Name  = name;
    Value = value;
  }

};

//-----------------------------------------------------------------------------
// Real
//-----------------------------------------------------------------------------

//! \ingroup MOBIUS_CORE
//!
//! Real variable.
class core_VarReal : public core_VarBase
{
public:

  //! Factory method ensuring allocation of Variable instance in a heap.
  //! \param[in] name  variable name.
  //! \param[in] value variable value.
  //! \return variable instance.
  static t_ptr<core_VarReal>
    Instance(const std::string& name,
             const double       value)
  {
    return new core_VarReal(name, value);
  }

public:

  //! Variable value.
  double Value;

public:

  //! Default constructor.
  core_VarReal() : core_VarBase()
  {}

  //! Constructor accepting variable value as an argument. The variable will
  //! have no name (empty name member).
  //! \param[in] value variable value.
  core_VarReal(const double theValue) : core_VarBase()
  {
    Value = theValue;
  }

  //! Constructor accepting variable name and value as arguments.
  //! \param[in] name  variable name.
  //! \param[in] value variable value.
  core_VarReal(const std::string& name,
               const double       value) : core_VarBase()
  {
    Name  = name;
    Value = value;
  }

};

//-----------------------------------------------------------------------------
// Real vector
//-----------------------------------------------------------------------------

//! \ingroup MOBIUS_CORE
//!
//! Real vector variable.
class core_VarRealVector : public core_VarBase
{
public:

  //! Factory method ensuring allocation of Variable instance in a heap.
  //! \param[in] name  variable name.
  //! \param[in] value variable value.
  //! \return variable instance.
  static t_ptr<core_VarRealVector>
    Instance(const std::string&         name,
             const std::vector<double>& value)
  {
    return new core_VarRealVector(name, value);
  }

public:

  //! Variable value.
  std::vector<double> Value;

public:

  //! Default constructor.
  core_VarRealVector() : core_VarBase()
  {}

  //! Constructor accepting variable value as an argument. The variable will
  //! have no name (empty name member).
  //! \param[in] value variable value.
  core_VarRealVector(const std::vector<double>& value) : core_VarBase()
  {
    Value = value;
  }

  //! Constructor accepting variable name and value as arguments.
  //! \param[in] name  variable name.
  //! \param[in] value variable value.
  core_VarRealVector(const std::string&         name,
                     const std::vector<double>& value) : core_VarBase()
  {
    Name  = name;
    Value = value;
  }

};

//-----------------------------------------------------------------------------
// Boolean
//-----------------------------------------------------------------------------

//! \ingroup MOBIUS_CORE
//!
//! Boolean variable.
class core_VarBool : public core_VarBase
{
public:

  //! Factory method ensuring allocation of Variable instance in a heap.
  //! \param[in] name  variable name.
  //! \param[in] value variable value.
  //! \return variable instance.
  static t_ptr<core_VarBool>
    Instance(const std::string& name,
             const bool         value)
  {
    return new core_VarBool(name, value);
  }

public:

  //! Variable value.
  bool Value;

public:

  //! Default constructor.
  core_VarBool() : core_VarBase()
  {}

  //! Constructor accepting variable value as an argument. The variable will
  //! have no name (empty name member).
  //! \param[in] value variable value.
  core_VarBool(const bool value) : core_VarBase()
  {
    Value = value;
  }

  //! Constructor accepting variable name and value as arguments.
  //! \param[in] name  variable name.
  //! \param[in] value variable value.
  core_VarBool(const std::string& name,
               const bool         value) : core_VarBase()
  {
    Name  = name;
    Value = value;
  }

};

//-----------------------------------------------------------------------------
// String
//-----------------------------------------------------------------------------

//! \ingroup MOBIUS_CORE
//!
//! String variable.
class core_VarString : public core_VarBase
{
public:

  //! Factory method ensuring allocation of Variable instance in a heap.
  //! \param[in] name  variable name.
  //! \param[in] value variable value.
  //! \return variable instance.
  static t_ptr<core_VarString>
    Instance(const std::string& name,
             const std::string& value)
  {
    return new core_VarString(name, value);
  }

public:

  //! Variable value.
  std::string Value;

public:

  //! Default constructor.
  core_VarString() : core_VarBase()
  {}

  //! Constructor accepting variable value as an argument. The variable will
  //! have no name (empty name member).
  //! \param[in] value variable value.
  core_VarString(const std::string& value) : core_VarBase()
  {
    Value = value;
  }

  //! Constructor accepting variable name and value as arguments.
  //! \param[in] name  variable name.
  //! \param[in] value variable value.
  core_VarString(const std::string& name,
                 const std::string& value) : core_VarBase()
  {
    Name  = name;
    Value = value;
  }

};

//-----------------------------------------------------------------------------
// Auxiliary
//-----------------------------------------------------------------------------

//! \ingroup MOBIUS_CORE
//!
//! Collection of variables.
typedef std::vector< t_ptr<core_VarBase> > core_Vars;

}

#endif
