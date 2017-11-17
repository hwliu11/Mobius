//-----------------------------------------------------------------------------
// Created on: 30 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef testEngine_DescriptionProc_HeaderFile
#define testEngine_DescriptionProc_HeaderFile

// testEngine includes
#include <mobius/testEngine_Macro.h>

// Core includes
#include <mobius/core.h>

// STD includes
#include <map>
#include <vector>

namespace mobius {

//! Utility class for processing of description files corresponding to Test
//! Cases.
class testEngine_DescriptionProc
{
public:

  //! shortcut for variable expansion maps.
  typedef std::map<std::string, std::string> StrStrMap;

  //! shortcut for string-string pairs.
  typedef std::pair<std::string, std::string> StrStrPair;

  //! Shortcut for ordered collection of strings.
  typedef std::vector<std::string> SeqStr;

public:

  mobiusTestEngine_EXPORT static bool
    Process(const std::string& dir,
            const std::string& filename,
            const StrStrMap& vars,
            const int caseID,
            std::string& title,
            SeqStr& overviewBlocks,
            SeqStr& detailsBlocks);

private:

  static void
    extractBlocks(const std::string& text,
                  std::string& title,
                  SeqStr& overviewBlocks,
                  SeqStr& detailsBlocks);

  static void
    extractIndicesFromTag(const std::string& tag,
                          int& startIdx,
                          int& endIdx);

  static std::string
    expandVariables(const std::string& text,
                    const StrStrMap& vars,
                    const std::string& varsScope);

  static bool
    isLineOfNulls(const std::string& line);

  static bool
    isVar(const std::string& token,
          int& varStart,
          int& varEnd);

  static bool
    isPre(const std::string& token,
          const bool isOpen);

  static std::string
    varScope(const int caseID,
             const int funcID);

private:

  testEngine_DescriptionProc() {};
  testEngine_DescriptionProc(const testEngine_DescriptionProc&) {};
  void operator=(const testEngine_DescriptionProc&) {}

};

};

#endif
