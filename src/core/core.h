//-----------------------------------------------------------------------------
// Created on: 28 August 2012
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

#ifndef core_HeaderFile
#define core_HeaderFile

// STD includes
#include <iomanip>
#include <iostream>
#include <limits>
#include <math.h>
#include <sstream>
#include <string>
#include <vector>

// ADOL-C
#if defined USE_ADOLC
  #pragma warning(push, 0)
    #include <adolc/adtl.h>
  #pragma warning(pop)
  using namespace adtl;
#else
  #define adouble double
#endif

#define _USE_MATH_DEFINES

#define fmin(a, b) (((a) < (b)) ? (a) : (b))
#define fmax(a, b) (((a) > (b)) ? (a) : (b))

#define core_NotUsed(x) x

#define SLASH_STR "\\"

#if defined _WIN32
  #if defined mobiusCore_EXPORTS
    #define mobiusCore_EXPORT __declspec(dllexport)
  #else
    #define mobiusCore_EXPORT __declspec(dllimport)
  #endif
#else
  #define mobiusCore_EXPORT
#endif

//-----------------------------------------------------------------------------
// DOXY group definition
//-----------------------------------------------------------------------------
//! \defgroup MOBIUS_CORE Core
//!
//! Core tools and data types.
//-----------------------------------------------------------------------------

#define MOBIUS_TEST_DATA    "MOBIUS_TEST_DATA"
#define MOBIUS_TEST_DUMPING "MOBIUS_TEST_DUMPING"
#define MOBIUS_TEST_DESCR   "MOBIUS_TEST_DESCR"

//-----------------------------------------------------------------------------
// Global auxiliary functions
//-----------------------------------------------------------------------------

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Auxiliary functions.
namespace core
{
  //! Functions for working with environment.
  namespace env
  {
    mobiusCore_EXPORT
      std::string MobiusTestData();

    mobiusCore_EXPORT
      std::string MobiusTestDumping();

    mobiusCore_EXPORT
      std::string MobiusTestDescr();

    mobiusCore_EXPORT
      std::string GetVariable(const char* VarName);
  }

  //! Functions for working with strings.
  namespace str
  {
    mobiusCore_EXPORT std::string
      slashed(const std::string& strIN);

    //! Extracts integer number from the passed string.
    //! \param str [in] input string.
    //! \returns extracted integer value.
    mobiusCore_EXPORT int
      extract_int(const std::string& str);

    //! Checks whether the two passed strings are equal.
    //! \param str_1 [in] first string.
    //! \param str_2 [in] second string.
    //! \return true/false.
    mobiusCore_EXPORT bool
      are_equal(const std::string& str_1,
                const std::string& str_2);

    //! Checks whether the passed string is number or not.
    //! \param str [in] string to check.
    //! \return true/false.
    mobiusCore_EXPORT bool
      is_number(const std::string& str);

    //! Replaces all occurrences of {what} with {with} in string {str}.
    //! \param str [in/out] target string.
    //! \param what [in] sub-string to replace.
    //! \param with [in] string to replace with.
    mobiusCore_EXPORT void
      replace_all(std::string&       str,
                  const std::string& what,
                  const std::string& with);

    //! Splits the passed string by the given delimiter. Note that the
    //! passed output vector is not cleaned up beforehand.
    //! \param source_str [in]  input string to split.
    //! \param delim_str  [in]  delimiter string.
    //! \param result     [out] resulting collection of tokens after split.
    mobiusCore_EXPORT void
      split(const std::string&        source_str,
            const std::string&        delim_str,
            std::vector<std::string>& result);

    //! Joins the elements of the given vector into one string.
    //! \param vector [in]  input vector.
    //! \param result [out] resulting string.
    //! \param from   [in]  index to start from.
    //! \param until  [in]  index to join until.
    mobiusCore_EXPORT void
      join(const std::vector<std::string>& vector,
           std::string&                    result,
           const int                       from = 0,
           const int                       until = -1);

    //! Extracts substring from the passed source.
    //! \param source [in] input string to extract substring from.
    //! \param idx_F  [in] 0-based index to start extraction from (inclusively).
    //! \param length [in] number of characters to extract.
    //! \return resulting substring.
    mobiusCore_EXPORT std::string
      substr(const std::string& source,
             const int          idx_F,
             const int          length);

    //! Attempts to separated the passed string into key vs value tuple
    //! using the given key string and separator.
    //! \param source [in]  input string in question.
    //! \param delim  [in]  delimeter.
    //! \param key    [in]  key string.
    //! \param value  [out] found value (if any).
    //! \return true if separation is possible, false -- otherwise.
    mobiusCore_EXPORT bool
      getKeyValue(const std::string& source,
                  const std::string& delim,
                  const std::string& key,
                  std::string&       value);

    //! Converts the passed value to string. This function is used to
    //! substitute std::to_string() for compilers incompatible with
    //! C++ 11.
    //! \param value [in] value to convert.
    //! \return string.
    template <typename T>
    static std::string to_string(T value)
    {
      std::ostringstream os;
      os << std::setprecision( std::numeric_limits<double>::max_digits10 );
      os << value;
      return os.str();
    }

    //! Converts the passed string to number.
    //! \param[in] str string to convert.
    //! \return string.
    template <typename T>
    static T to_number(const std::string& str)
    {
      std::istringstream is(str);
      T result;
      is >> result;
      return result;
    }
  }

  //! Hashers.
  namespace hasher
  {
    //! Computes hash code for integer value.
    //! \param[in] val   value in question.
    //! \param[in] upper upper bound.
    //! \return hash code.
    mobiusCore_EXPORT int
      HashCode(const int val,
               const int upper);

    //! Computes hash code for double value.
    //! \param[in] val   value in question.
    //! \param[in] upper upper bound.
    //! \return hash code.
    mobiusCore_EXPORT int
      HashCode(const double val,
               const int    upper);
  }

}

}

#endif
