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

#ifndef core_Timer_HeaderFile
#define core_Timer_HeaderFile

// Core includes
#include <mobius/core_Chronometer.h>
#include <mobius/core_MemChecker.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Utility to measure execution time.
class core_Timer : public core_Chronometer
{
public:

  //! Builds a timer initialized and stopped.
  //! @param theThisThreadOnly when TRUE, measured CPU time will account time of the current thread only;
  //!                          otherwise CPU of the process (all threads, and completed children) is measured;
  //!                          this flag does NOT affect ElapsedTime() value, only values returned by core_Chronometer
  mobiusCore_EXPORT
    core_Timer(bool theThisThreadOnly = false);

public:

  //! Stops and reinitializes the timer with specified elapsed time.
  mobiusCore_EXPORT void
    Reset(const double theTimeElapsedSec);

  //! Stops and reinitializes the timer with zero elapsed time.
  mobiusCore_EXPORT virtual void
    Reset() override;

  //! Restarts the Timer.
  mobiusCore_EXPORT virtual void
    Restart() override;

  //! Shows both the elapsed time and CPU time on the standard output
  //! stream <cout>. The chronometer can be running (Lap Time) or
  //! stopped.
  mobiusCore_EXPORT virtual void
    Show() const override;

  //! Shows both the elapsed time and CPU  time on the
  //! output stream <OS>.
  mobiusCore_EXPORT virtual void
    Show(std::ostream& os) const override;

  //! returns both the elapsed time(seconds,minutes,hours)
  //! and CPU  time.
  mobiusCore_EXPORT void
    Show(double& theSeconds, int& theMinutes, int& theHours, double& theCPUtime) const;

  //! Stops the Timer.
  mobiusCore_EXPORT virtual void
    Stop() override;

  //! Starts (after Create or Reset) or restarts (after Stop)
  //! the Timer.
  mobiusCore_EXPORT virtual void
    Start() override;

  //! Returns elapsed time in seconds.
  mobiusCore_EXPORT double
    ElapsedTime() const;

private:

  double m_fTimeStart;
  double m_fTimeCumul;

};

};

//-----------------------------------------------------------------------------
//                           MEASURING PERFORMANCE
//-----------------------------------------------------------------------------

//! Example:
//!
//! #ifdef MOBIUS_TIMER_NEW
//!   MOBIUS_TIMER_NEW
//!   MOBIUS_TIMER_RESET
//!   MOBIUS_TIMER_GO
//! #endif
//!
//! ... YOUR ALGO GOES HERE ...
//!
//! #ifdef MOBIUS_TIMER_NEW
//!   MOBIUS_TIMER_FINISH
//!   MOBIUS_TIMER_COUT_RESULT
//! #endif

#define MOBIUS_TIMER_NEW \
  core_Timer __aux_debug_Timer; \
  double __aux_debug_Seconds, __aux_debug_CPUTime; \
  int __aux_debug_Minutes, __aux_debug_Hours; \
  int __aux_debug_memcheck_before, __aux_debug_memcheck_after;

#define MOBIUS_TIMER_RESET \
  __aux_debug_Seconds = __aux_debug_CPUTime = 0.0; \
  __aux_debug_Minutes = __aux_debug_Hours = 0; \
  __aux_debug_Timer.Reset();

#define MOBIUS_TIMER_GO \
  MOBIUS_MEMCHECK_COUNT_MIB(__aux_debug_memcheck_before) \
  __aux_debug_Timer.Start();

#define MOBIUS_TIMER_FINISH \
  MOBIUS_MEMCHECK_COUNT_MIB(__aux_debug_memcheck_after) \
  __aux_debug_Timer.Stop(); \
  __aux_debug_Timer.Show(__aux_debug_Seconds, __aux_debug_Minutes, __aux_debug_Hours, __aux_debug_CPUTime);

#define MOBIUS_TIMER_COUT_RESULT \
  { \
    TIMER_COUT_RESULT_MSG(""); \
  }

#define MOBIUS_TIMER_COUT_RESULT_MSG(Msg) \
  { \
    std::cout << "\n=============================================" << std::endl; \
    std::string ascii_msg(Msg); \
    if ( !ascii_msg.empty() ) \
    { \
      std::cout << Msg                                             << std::endl; \
      std::cout << "---------------------------------------------" << std::endl; \
    } \
    std::cout << "Seconds:  " << __aux_debug_Seconds               << std::endl; \
    std::cout << "Minutes:  " << __aux_debug_Minutes               << std::endl; \
    std::cout << "Hours:    " << __aux_debug_Hours                 << std::endl; \
    std::cout << "CPU time: " << __aux_debug_CPUTime               << std::endl; \
    std::cout << "=============================================\n" << std::endl; \
  }

#define MOBIUS_TIMER_COUT_RESULT_NOTIFIER(Notifier, Msg) \
  { \
    Notifier->SendLogMessage(MobiusInfo(Normal) << "============================================="); \
    Notifier->SendLogMessage(MobiusInfo(Normal) << "%1" << Msg); \
    Notifier->SendLogMessage(MobiusInfo(Normal) << "---------------------------------------------"); \
    Notifier->SendLogMessage(MobiusInfo(Normal) << "\tElapsed time (seconds):  %1"     << __aux_debug_Seconds); \
    Notifier->SendLogMessage(MobiusInfo(Normal) << "\tElapsed time (minutes):  %1"     << __aux_debug_Minutes); \
    Notifier->SendLogMessage(MobiusInfo(Normal) << "\tElapsed time (hours):    %1"     << __aux_debug_Hours); \
    Notifier->SendLogMessage(MobiusInfo(Normal) << "\tElapsed time (CPU time): %1"     << __aux_debug_CPUTime); \
    Notifier->SendLogMessage(MobiusInfo(Normal) << "\tMemory before:           %1 MiB" << __aux_debug_memcheck_before); \
    Notifier->SendLogMessage(MobiusInfo(Normal) << "\tMemory after [delta]:    %1 MiB [%2]" \
                                                << __aux_debug_memcheck_after \
                                                << (__aux_debug_memcheck_after - __aux_debug_memcheck_before) ); \
    Notifier->SendLogMessage(MobiusInfo(Normal) << "... Finished."); \
  }

#endif // core_Timer_HeaderFile
