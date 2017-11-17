//-----------------------------------------------------------------------------
// Created on: 11 June 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef testEngine_TestCase_HeaderFile
#define testEngine_TestCase_HeaderFile

// testEngine includes
#include <mobius/testEngine_Macro.h>

// Core includes
#include <mobius/core_XYZ.h>

// STD includes
#include <map>
#include <vector>

//-----------------------------------------------------------------------------
// Auxiliary definitions
//-----------------------------------------------------------------------------

#define DEFINE_TEST_VARIABLES \
  mobius::testEngine_TestCase::StrStrMap mobius::testEngine_TestCase::m_varExpansion;

namespace mobius {

//! Pointer to Test Function.
//! Please note that {funcID} should be normally passed by Test Case. The
//! convention is to have {funcID} as 1-based integer number.
typedef bool (*MobiusTestFunction)(const int funcID);

//! Collection of pointers to Test Functions
class MobiusTestFunctions
{
public:

  //! Default constructor.
  MobiusTestFunctions() {}

public:

  //! Adds the passed function pointer to the collection of pointers to
  //! Test Functions.
  //! \param funcPtr [in] function pointer to add.
  //! \return this reference for subsequent streaming.
  MobiusTestFunctions& operator<<(const MobiusTestFunction& funcPtr)
  {
    m_testFunctions.push_back(funcPtr);
    return*this;
  }

  //! Returns size of the collection of function pointers.
  //! \return requested size.
  inline size_t Size() const
  {
    return m_testFunctions.size();
  }

  //! Returns function pointer for the given index.
  //! \param idx [in] index of the function pointer to access.
  //! \return requested function pointer.
  const MobiusTestFunction& Func(const int idx) const
  {
    return m_testFunctions.at(idx);
  }

private:

  //! Internal collection of pointers to Test Functions.
  std::vector<MobiusTestFunction> m_testFunctions;

};

//-----------------------------------------------------------------------------
// Base class for Test Cases
//-----------------------------------------------------------------------------

//! Base class for Test Cases.
class testEngine_TestCase
{
public:

  //! shortcut for variable expansion maps.
  typedef std::map<std::string, std::string> StrStrMap;

  //! shortcut for string-string pairs.
  typedef std::pair<std::string, std::string> StrStrPair;

// Verification:
public:

  //! Compares two matrices on exact equality of their elements in the
  //! given 2-dimensional range. Notice that safety of such comparison is
  //! not guaranteed, so make sure that the compared matrices have equal
  //! dimensions.
  //! \param mx     [in] first matrix to compare.
  //! \param mx_ref [in] second matrix to compare.
  //! \param nRows  [in] number of rows to compare.
  //! \param nCols  [in] number of columns to compare.
  //! \return true in case of equality, false -- otherwise.
  static bool CompareExact(double**  mx,
                           double**  mx_ref,
                           const int nRows,
                           const int nCols)
  {
    for ( int i = 0; i < nRows; ++i )
      for ( int j = 0; j < nCols; ++j )
        if ( mx[i][j] != mx_ref[i][j] )
          return false;

    return true;
  }

// Expansion of variables:
public:

  //! Expands variable with the given name to the given value.
  //! \param varName  [in] variable name.
  //! \param varValue [in] variable value.
  //! \param caseID   [in] ID of the Test Case.
  //! \param funcID   [in] 1-based ID of Test Function to be used as a namespace
  //!                      for its own local variables mapping.
  static void SetVarDescr(const std::string& varName,
                          const std::string& varValue,
                          const int          caseID,
                          const int          funcID)
  {
    std::string vName = generateVarName(varName, caseID, funcID);

    if ( m_varExpansion.find(vName) != m_varExpansion.end() )
      m_varExpansion.erase(vName);

    m_varExpansion.insert( StrStrPair(vName, varValue) );
  }

  //! Overloaded variable setter.
  //! \param varName [in] variable name.
  //! \param val     [in] variable value.
  //! \param caseID  [in] ID of the Test Case.
  //! \param funcID  [in] 1-based ID of Test Function to be used as a namespace
  //!                     for its own local variables mapping.
  template <typename VAR_TYPE>
  static void SetVarDescr(const std::string& varName,
                          const VAR_TYPE     val,
                          const int          caseID,
                          const int          funcID)
  {
    SetVarDescr(varName, core::to_string(val), caseID, funcID);
  }

  //! Overloaded variable setter for Boolean values.
  //! \param varName [in] variable name.
  //! \param isYes   [in] variable value.
  //! \param caseID  [in] ID of the Test Case.
  //! \param funcID  [in] 1-based ID of Test Function to be used as a namespace
  //!                     for its own local variables mapping.
  static void SetVarDescr(const std::string& varName,
                          const bool         isYes,
                          const int          caseID,
                          const int          funcID)
  {
    SetVarDescr(varName, isYes ? std::string("yes") : std::string("no"), caseID, funcID);
  }

  //! Overloaded variable setter for XYZ values.
  //! \param varName [in] variable name.
  //! \param XYZ     [in] variable value.
  //! \param caseID  [in] ID of the Test Case.
  //! \param funcID  [in] 1-based ID of Test Function to be used as a namespace
  //!                     for its own local variables mapping.
  static void SetVarDescr(const std::string& varName,
                          const xyz&         XYZ,
                          const int          caseID,
                          const int          funcID)
  {
    std::string xyzStr = std::string("[")
                    + core::to_string( XYZ.X() ) + ", "
                    + core::to_string( XYZ.Y() ) + ", "
                    + core::to_string( XYZ.Z() )
                    + std::string("]");

    SetVarDescr(varName, xyzStr, caseID, funcID);
  }

  //! Overloaded variable setter for real array.
  //! \param varName [in] variable name.
  //! \param arr     [in] array as a value.
  //! \param n       [in] number of elements in the array.
  //! \param caseID  [in] ID of the Test Case.
  //! \param funcID  [in] 1-based ID of Test Function to be used as a namespace
  //!                     for its own local variables mapping.
  static void SetVarDescr(const std::string& varName,
                          const double*      arr,
                          const int          n,
                          const int          caseID,
                          const int          funcID)
  {
    std::string arrStr("[");
    if ( n )
    {
      for ( int i = 0; i < n; ++i )
      {
        arrStr += core::to_string(arr[i]);
        if ( i != (n - 1) )
          arrStr += ", ";
      }
    }
    arrStr += "]";

    SetVarDescr(varName, arrStr, caseID, funcID);
  }

  //! Overloaded variable setter for a vector.
  //! \param varName [in] variable name.
  //! \param vec     [in] vector as a value.
  //! \param caseID  [in] ID of the Test Case.
  //! \param funcID  [in] 1-based ID of Test Function to be used as a namespace
  //!                     for its own local variables mapping.
  template <typename ELEM_TYPE>
  static void SetVarDescr(const std::string&            varName,
                          const std::vector<ELEM_TYPE>& vec,
                          const int                     caseID,
                          const int                     funcID)
  {
    std::string arrStr("[");
    if ( vec.size() )
    {
      for ( int i = 0; i < vec.size(); ++i )
      {
        arrStr += core::to_string(vec[i]);
        if ( i != (vec.size() - 1) )
          arrStr += ", ";
      }
    }
    arrStr += "]";

    SetVarDescr(varName, arrStr, caseID, funcID);
  }

  //! Overloaded variable setter for real matrix.
  //! \param varName [in] variable name.
  //! \param mx      [in] matrix as a value.
  //! \param nRows   [in] number of rows in the matrix.
  //! \param nCols   [in] number of columns in the matrix.
  //! \param caseID  [in] ID of the Test Case.
  //! \param funcID  [in] 1-based ID of Test Function to be used as a namespace
  //!                     for its own local variables mapping.
  static void SetVarDescr(const std::string& varName,
                          double**           mx,
                          const int          nRows,
                          const int          nCols,
                          const int          caseID,
                          const int          funcID)
  {
    std::string mxStr;
    if ( nRows )
    {
      for ( int i = 0; i < nRows; ++i )
      {
        mxStr += "[";
        for ( int j = 0; j < nCols; ++j )
        {
          mxStr += core::to_string(mx[i][j]);
          if ( j != (nCols - 1) )
            mxStr += ", ";
        }
        mxStr += "]";
        if ( i != (nRows - 1) )
          mxStr += ", ";
      }
    }

    SetVarDescr(varName, mxStr, caseID, funcID);
  }

  //! Returns variables expansion map.
  //! \return expansion map.
  static const StrStrMap& ExpansionMap()
  {
    return m_varExpansion;
  }

private:

  //! Returns variable name using namespace of the Test Function.
  //! \param varName [in] short name.
  //! \param caseID  [in] ID of the Test Case.
  //! \param funcID  [in] 1-based ID of Test Function to be used as a namespace
  //!                     for its own local variables mapping.
  //! \return long variable name which is unique in scope of Test Function.
  static std::string generateVarName(const std::string& varName,
                                     const int          caseID,
                                     const int          funcID)
  {
    return core::to_string(caseID) + testEngine_Macro_NAMESPACE +
           core::to_string(funcID) + testEngine_Macro_NAMESPACE + varName;
  }

private:

  //! Local map for expansion of variables which can be used in
  //! descriptions of the Test Functions.
  static StrStrMap m_varExpansion;

};

};

#endif
