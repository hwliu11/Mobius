//-----------------------------------------------------------------------------
// Created on: 11 June 2013
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

#ifndef testEngine_Launcher_HeaderFile
#define testEngine_Launcher_HeaderFile

// testEngine includes
#include <mobius/testEngine_TestCase.h>

// Core includes
#include <mobius/core_IProgressNotifier.h>
#include <mobius/core_Ptr.h>

//-----------------------------------------------------------------------------
// Launcher API
//-----------------------------------------------------------------------------

namespace mobius {

//! Base class for Test Case Launcher.
class testEngine_CaseLauncherAPI : public core_OBJECT
{
public:

  virtual bool
    Launch() = 0;

  virtual int
    CaseID() const = 0;

  virtual std::string
    CaseDescriptionDir() const = 0;

  virtual std::string
    CaseDescriptionFn() const = 0;

  virtual int
    NumberOfExecuted() const = 0;

  virtual int
    NumberOfFailed() const = 0;

  virtual MobiusTestFunction
    TestFunction(const int idx) const = 0;

  virtual const std::map<std::string, std::string>&
    Variables() const = 0;

  virtual bool
    IsPassed(const int idx) const = 0;

};

//-----------------------------------------------------------------------------
// Launcher for single Test Case
//-----------------------------------------------------------------------------

//! Template-based implementation of Launcher mechanism dedicated to certain
//! Test Case.
template <typename CaseType>
class testEngine_CaseLauncher : public testEngine_CaseLauncherAPI
{
public:

  //! Constructor.
  //! \param[in] progress progress entry.
  testEngine_CaseLauncher(core_ProgressEntry progress)
  : testEngine_CaseLauncherAPI (),
    m_progress                 (progress)
  {}

public:

  //! Launches the Test Case of the given type. Returns true in case of
  //! success, false -- otherwise.
  //! \return true/false.
  virtual bool Launch()
  {
    // Collect Test Functions to run.
    MobiusTestFunctions functions;
    CaseType::Functions(functions);

    // Run functions one by one.
    bool areAllOk = true;
    for ( int f = 0; f < (int) functions.Size(); ++f )
    {
      const MobiusTestFunction& func = functions.Func(f);

      // Run test function.
      outcome res = ( (*func)(f + 1) );
      //
      const bool isOk = res.ok;

      // Dump summary.
      m_progress.SendLogMessage(MobiusNotice(Normal) << "Summary on test function %1: "
                                                     << res.name);
      m_progress.SendLogMessage(MobiusNotice(Normal) << "\t\tIs ok: %1: "
                                                     << isOk);
      m_progress.SendLogMessage(MobiusNotice(Normal) << "\t\tElapsed time: %1 [sec]: "
                                                     << res.elapsedTimeSec);
      m_progress.SendLogMessage(MobiusNotice(Normal) << "\t\tMemory footprint: %1 [MiB]: "
                                                     << (res.memAfter - res.memBefore));
      //
      std::cout << "..." << std::endl;

      // Store the result.
      m_funcResults.push_back(isOk);
      //
      if ( !isOk && areAllOk )
        areAllOk = false;
    }
    return areAllOk;
  }

  //! Returns ID of the managed Test Case.
  //! \return ID of the managed Test Case.
  virtual int CaseID() const
  {
    return CaseType::ID();
  }

  //! Returns description directory for the managed Test Case.
  //! \return description directory for the managed Test Case.
  virtual std::string CaseDescriptionDir() const
  {
    return CaseType::DescriptionDir();
  }

  //! Returns description filename for the managed Test Case.
  //! \return description filename for the managed Test Case.
  virtual std::string CaseDescriptionFn() const
  {
    return CaseType::DescriptionFn();
  }

  //! Returns the total number of executed Test Functions for the managed
  //! Test Case.
  //! \return number of executed Test Functions
  virtual int NumberOfExecuted() const
  {
    return (int) m_funcResults.size();
  }

  //! Returns the total number of failed Test Functions for the managed
  //! Test Case.
  //! \return number of failed Test Functions
  virtual int NumberOfFailed() const
  {
    int numFailed = 0;
    for ( int f = 0; f < (int) m_funcResults.size(); ++f )
    {
      if ( !m_funcResults[f] )
        numFailed++;
    }
    return numFailed;
  }

  //! Returns Test Function referred to by the given 0-based index.
  //! \param idx [in] index of the Test Function to access.
  //! \return Test Function.
  virtual MobiusTestFunction TestFunction(const int idx) const
  {
    // Collect Test Functions to run
    MobiusTestFunctions functions;
    CaseType::Functions(functions);

    // Access Test Function by index
    return functions.Func(idx);
  }

  //! Returns expansion map for local description variables of Test
  //! Function with the given index.
  //! \return expansion map.
  virtual const std::map<std::string, std::string>& Variables() const
  {
    return CaseType::ExpansionMap();
  }

  //! Returns true if Test Function with the given index has passed, false --
  //! otherwise.
  //! \param idx [in] index of Test Function.
  //! \return true/false.
  virtual bool IsPassed(const int idx) const
  {
    return m_funcResults[idx];
  }

private:

  testEngine_CaseLauncher(const testEngine_CaseLauncher&) {}
  void operator=(const testEngine_CaseLauncher&) {}

private:

  std::vector<bool>  m_funcResults; //!< Execution results.
  core_ProgressEntry m_progress;    //!< Progress entry.

};

//-----------------------------------------------------------------------------
// Launcher for entire test suite
//-----------------------------------------------------------------------------

//! Launcher for entire test suite. The instance of this class should be
//! populated with the actual Test Case Launchers you want to execute.
class testEngine_Launcher
{
public:

  //! Default constructor.
  testEngine_Launcher() {}

public:

  mobiusTestEngine_EXPORT testEngine_Launcher&
    operator<<(const t_ptr<testEngine_CaseLauncherAPI>& CaseLauncher);

  mobiusTestEngine_EXPORT bool
    Launch(std::ostream* out = nullptr) const;

protected:

  bool generateReport(std::ostream* out) const;
  std::string uniqueDirName() const;

private:

  testEngine_Launcher(const testEngine_Launcher&) {}
  void operator=(const testEngine_Launcher&) {}

private:

  //! Internal collection of Test Case Launchers.
  std::vector< t_ptr<testEngine_CaseLauncherAPI> > m_launchers;

};

};

#endif
