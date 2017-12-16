//-----------------------------------------------------------------------------
// Created on: 13 October 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
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
