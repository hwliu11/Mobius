//-----------------------------------------------------------------------------
// Created on: 13 October 2013
//-----------------------------------------------------------------------------
// Copyright (c) 2017, Sergey Slyadnev
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
#include <mobius/core_Utils.h>

// OS dependent
#include <windows.h>

#define BUFSIZE         1000
#define QR_DATA         "QR_DATA"
#define QR_TEST_DUMPING "QR_TEST_DUMPING"
#define QR_TEST_DESCR   "QR_TEST_DESCR"

const char* SLASH_STR = "\\";

//! Returns value of QR_DATA environment variable. This variable is used to
//! refer to the directory containing all data files playing as inputs for
//! unit tests.
//! \return value of QR_DATA variable.
std::string mobius::core_Utils::Env::QrData()
{
  return GetVariable(QR_DATA);
}

//! Returns value of QR_TEST_DUMPING environment variable. This variable is
//! used to refer to the directory containing all temporary files utilized by
//! unit tests.
//! \return value of QR_TEST_DUMPING variable.
std::string mobius::core_Utils::Env::QrDumping()
{
  return GetVariable(QR_TEST_DUMPING);
}

//! Returns value of QR_TEST_DESCR environment variable. This variable is used
//! to refer to the directory containing all temporary files utilized by unit
//! tests.
//! \return value of QR_DUMPING variable.
std::string mobius::core_Utils::Env::QrDescr()
{
  return GetVariable(QR_TEST_DESCR);
}

//! Returns value of the environment variable with the passed name.
//! \param VarName [in] variable name.
//! \return value of the variable.
std::string mobius::core_Utils::Env::GetVariable(const char* VarName)
{
  TCHAR chNewEnv[BUFSIZE];
  GetEnvironmentVariable(VarName, chNewEnv, BUFSIZE);
  return chNewEnv;
}

//! Adds slash character to the passed string. The original string is not
//! changed.
//! \param strIN [in] input string to append slash to.
//! \return string with a trailing slash.
std::string mobius::core_Utils::Str::Slashed(const std::string& strIN)
{
  char c = strIN.at(strIN.length() - 1);
  if ( c == *SLASH_STR )
    return strIN;

  std::string strOUT(strIN);
  strOUT.append(SLASH_STR);
  return strOUT;
}
