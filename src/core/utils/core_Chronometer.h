//-----------------------------------------------------------------------------
// Created on: 25 July 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2018-present, Sergey Slyadnev
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

#ifndef core_Chronometer_HeaderFile
#define core_Chronometer_HeaderFile

// Core includes
#include <mobius/core.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Utilitiy to measure CPU time (both user and system) consumed by the
//! current process or thread. The chronometer can be started and stopped
//! multiple times, and measures cumulative time.
class core_Chronometer 
{
public:

  //! Initializes a stopped Chronometer.
  //!
  //! If useThisThreadOnly is true, the measured CPU time will account
  //! time of the current thread only; otherwise CPU of the
  //! process (all threads, and completed children) is measured.
  mobiusCore_EXPORT
    core_Chronometer(const bool useThisThreadOnly = false);

  //! Destructor.
  mobiusCore_EXPORT virtual
    ~core_Chronometer();

public:

  //! Return true if timer has been started.
  bool IsStarted() const
  {
    return !m_bIsStopped;
  }

  //! Returns the current CPU user time in seconds.
  //! The chronometer can be running (laps Time) or stopped.
  double UserTimeCPU() const
  {
    double aUserTime = 0.0, aSysTime = 0.0;
    Show (aUserTime, aSysTime);
    return aUserTime;
  }

  //! Returns the current CPU system time in seconds.
  //! The chronometer can be running (laps Time) or stopped.
  double SystemTimeCPU() const
  {
    double aUserTime = 0.0, aSysTime = 0.0;
    Show (aUserTime, aSysTime);
    return aSysTime;
  }

  //! Returns the current CPU user time in a variable.
  //! The chronometer can be running (laps Time) or stopped.
  void Show(double& theUserSeconds) const
  {
    theUserSeconds = UserTimeCPU();
  }

public:

  //! Stops and Reinitializes the Chronometer.
  mobiusCore_EXPORT virtual void
    Reset();

  //! Restarts the Chronometer.
  mobiusCore_EXPORT virtual void
    Restart();

  //! Stops the Chronometer.
  mobiusCore_EXPORT virtual void
    Stop();
  
  //! Starts (after Create or Reset) or restarts (after Stop)
  //! the chronometer.
  mobiusCore_EXPORT virtual void
    Start();

  //! Shows the current CPU user and system time.
  //! The chronometer can be running (laps Time) or stopped.
  mobiusCore_EXPORT virtual void
    Show() const;

  //! Shows the current CPU user and system time.
  //! The chronometer can be running (laps Time) or stopped.
  mobiusCore_EXPORT virtual void
    Show(std::ostream& out) const;

  //! Returns the current CPU user and system time in variables.
  //! The chronometer can be running (laps Time) or stopped.
  mobiusCore_EXPORT void
    Show(double& theUserSec, double& theSystemSec) const;

public:

  //! Returns CPU time (user and system) consumed by the current
  //! process since its start, in seconds. The actual precision of
  //! the measurement depends on granularity provided by the system,
  //! and is platform-specific.
  mobiusCore_EXPORT static void
    GetProcessCPU(double& UserSeconds, double& SystemSeconds);
  
  //! Returns CPU time (user and system) consumed by the current
  //! thread since its start. Note that this measurement is
  //! platform-specific, as threads are implemented and managed
  //! differently on different platforms and CPUs.
  mobiusCore_EXPORT static void
    GetThreadCPU(double& UserSeconds, double& SystemSeconds);

protected:

  double m_fStartCpuUser;
  double m_fStartCpuSys;
  double m_fCumulCpuUser;
  double m_fCumulCpuSys;
  bool   m_bIsStopped;
  bool   m_bIsThreadOnly;

};

}

#endif
