//-----------------------------------------------------------------------------
// Created on: 28 August 2012
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

#ifndef core_HeaderFile
#define core_HeaderFile

// STD includes
#include <iostream>
#include <math.h>
#include <sstream>
#include <string>
#include <vector>

#define _USE_MATH_DEFINES

#define fmin(a, b) (((a) < (b)) ? (a) : (b))
#define fmax(a, b) (((a) > (b)) ? (a) : (b))

#define core_NotUsed(x) x

#if defined _WIN32
  #if defined mobiusCore_EXPORTS
    #define mobiusCore_EXPORT __declspec(dllexport)
  #else
    #define mobiusCore_EXPORT __declspec(dllimport)
  #endif
#else
  #define mobiusCore_EXPORT
#endif

//const char* SLASH_STR = "\\";

//-----------------------------------------------------------------------------
// DOXY group definition
//-----------------------------------------------------------------------------
//! \defgroup MOBIUS_CORE Core
//!
//! Core tools and data types.
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Global auxiliary functions
//-----------------------------------------------------------------------------

//! \ingroup MOBIUS_CORE
//!
//! Auxiliary functions.
class core
{
public:

  //std::string slashed(const std::string& str)
  //{
  //  char c = str.at(str.length() - 1);
  //  if ( c == *SLASH_STR )
  //    return str;

  //  std::string strOUT(str);
  //  strOUT.append(SLASH_STR);
  //  return strOUT;
  //}

  //! Extracts integer number from the passed string.
  //! \param str [in] input string.
  //! \returns extracted integer value.
  static int extract_int(const std::string& str)
  {
    std::string temp;
    int number = 0;

    for ( unsigned int i = 0; i < str.size(); ++i )
    {
      // Iterate the string to find the first "number" character.
      // If found, start another loop to extract it
      // and then break the current one
      // thus extracting the first encountered numeric block
      if ( isdigit(str[i]) )
      {
        for ( unsigned int a = i; a < str.size(); ++a )
        {
          temp += str[a];
        }
        break; // The first numeric block is extracted
      }
    }

    std::istringstream stream(temp);
    stream >> number;
    return number;
  }

  //! Converts the passed value to string. This function is used to
  //! substitute std::to_string() for compilers incompatible with
  //! C++ 11.
  //! \param value [in] value to convert.
  //! \return string.
  template <typename T>
  static std::string to_string(T value)
  {
    std::ostringstream os;
    os << value;
    return os.str();
  }

  //! Checks whether the two passed strings are equal.
  //! \param str_1 [in] first string.
  //! \param str_2 [in] second string.
  //! \return true/false.
  static bool are_equal(const std::string& str_1,
                        const std::string& str_2)
  {
    return !str_1.compare(str_2);
  }

  //! Checks whether the passed string is number or not.
  //! \param str [in] string to check.
  //! \return true/false.
  static bool is_number(const std::string& str)
  {
    char* cnv;
    strtod(str.c_str(), &cnv);
    return cnv != str.data();
  }

  //! Converts the passed string to number.
  //! \param str [in] string to convert.
  //! \param default_value [in] default value to use.
  //! \return string.
  template <typename T>
  static T to_number(const std::string& str,
                     const T default_value = 0)
  {
    std::istringstream is(str);
    T result;
    (is >> result) ? result : default_value;
    return result;
  }

  //! Replaces all occurrences of {what} with {with} in string {str}.
  //! \param str [in/out] target string.
  //! \param what [in] sub-string to replace.
  //! \param with [in] string to replace with.
  static void replace_all(std::string&       str,
                          const std::string& what,
                          const std::string& with)
  {
    for ( size_t pos = 0; ; pos += with.length() )
    {
      pos = str.find(what, pos); // Locate the substring to replace
      if ( pos == std::string::npos )
        break; // Not found

      // Replace by erasing and inserting
      str.erase( pos, what.length() );
      str.insert(pos, with);
    }
  }

  //! Splits the passed string by the given delimiter. Note that the
  //! passed output vector is not cleaned up beforehand.
  //! \param source_str [in]  input string to split.
  //! \param delim_str  [in]  delimiter string.
  //! \param result     [out] resulting collection of tokens after split.
  static void split(const std::string&        source_str,
                    const std::string&        delim_str,
                    std::vector<std::string>& result)
  {
    // Initialize collection of strings to split
    std::vector<std::string> chunks;
    chunks.push_back(source_str);

    // Split by each delimiter consequently
    for ( size_t delim_idx = 0; delim_idx < delim_str.length(); ++delim_idx )
    {
      std::vector<std::string> new_chunks;
      const char delim = delim_str[delim_idx];

      // Split each chunk
      for ( size_t chunk_idx = 0; chunk_idx < chunks.size(); ++chunk_idx )
      {
        const std::string& source = chunks[chunk_idx];
        std::string::size_type currPos = 0, prevPos = 0;
        while ( (currPos = source.find(delim, prevPos) ) != std::string::npos )
        {
          std::string item = source.substr(prevPos, currPos - prevPos);
          if ( item.size() > 0 )
          {
            new_chunks.push_back(item);
          }
          prevPos = currPos + 1;
        }
        new_chunks.push_back( source.substr(prevPos) );
      }

      // Set new collection of chunks for splitting by the next delimiter
      chunks = new_chunks;
    }

    // Set result
    result = chunks;
  }

  //! Joins the elements of the given vector into one string.
  //! \param vector [in]  input vector.
  //! \param result [out] resulting string.
  //! \param from   [in]  index to start from.
  //! \param until  [in]  index to join until.
  static void join(const std::vector<std::string>& vector,
                   std::string&                    result,
                   const int                       from = 0,
                   const int                       until = -1)
  {
    std::string res;
    const int last = ( (until == -1) ? (int) vector.size() : until );
    for ( int idx = from; idx < last; ++idx )
      res += vector[idx];

    result = res;
  }

  //! Extracts substring from the passed source.
  //! \param source [in] input string to extract substring from.
  //! \param idx_F  [in] 0-based index to start extraction from (inclusively).
  //! \param length [in] number of characters to extract.
  //! \return resulting substring.
  static std::string substr(const std::string& source,
                            const int          idx_F,
                            const int          length)
  {
    return source.substr(idx_F, length);
  }

};

#endif
