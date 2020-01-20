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

#include <mobius/core_Timer.h>

#ifndef _WIN32

#include <sys/time.h>

//=======================================================================
//function : GetWallClockTime
//purpose  : Get current time in seconds with system-defined precision
//=======================================================================

static inline double GetWallClockTime ()
{
  struct timeval tv;
  // use time of first call as base for computing total time,
  // to avoid loss of precision due to big values of tv_sec (counted since 1970)
  static time_t startSec = (gettimeofday (&tv, nullptr) ? 0 : tv.tv_sec);
  return gettimeofday (&tv, nullptr) ? 0. : (tv.tv_sec - startSec) + 0.000001 * tv.tv_usec;
}

#else

#ifndef NOMINMAX
  #define NOMINMAX
#endif
//
#include <windows.h>

//=======================================================================
//function : GetWallClockTime
//purpose  : Get current time in seconds with system-defined precision
//=======================================================================

static inline double GetWallClockTime ()
{
  // compute clock frequence on first call
  static LARGE_INTEGER freq;
  static BOOL isOk = QueryPerformanceFrequency (&freq);

  LARGE_INTEGER time;
  return isOk && QueryPerformanceCounter (&time) ? 
         (double)time.QuadPart / (double)freq.QuadPart :
#ifndef OCCT_UWP
         0.001 * GetTickCount();
#else
         0.001 * GetTickCount64();
#endif
}

#endif /* _WIN32 */

namespace
{
  //! Auxiliary function splits elapsed time in seconds into Hours, Minutes and Seconds.
  //! @param theTimeSec [in]  elapsed time in seconds
  //! @param theHours   [out] clamped elapsed hours
  //! @param theMinutes [out] clamped elapsed minutes within range [0, 59]
  //! @param theSeconds [out] clamped elapsed seconds within range [0, 60)
  static void timeToHoursMinutesSeconds (double  theTimeSec,
                                         int&    theHours,
                                         int&    theMinutes,
                                         double& theSeconds)
  {
    int aSec = (int)theTimeSec;
    theHours   = aSec / 3600;
    theMinutes = (aSec - theHours * 3600) / 60;
    theSeconds = theTimeSec - theHours * 3600 - theMinutes * 60;
  }
}

//-----------------------------------------------------------------------------

mobius::core_Timer::core_Timer(bool theThisThreadOnly)
: core_Chronometer (theThisThreadOnly),
  m_fTimeStart     (0.0),
  m_fTimeCumul     (0.0)
{
  //
}

//-----------------------------------------------------------------------------

void mobius::core_Timer::Reset (const double theTimeElapsedSec)
{
  m_fTimeStart = 0.0;
  m_fTimeCumul = theTimeElapsedSec;
  core_Chronometer::Reset();
}

//-----------------------------------------------------------------------------

void mobius::core_Timer::Reset ()
{
  m_fTimeStart = m_fTimeCumul = 0.0;
  core_Chronometer::Reset();
}

//-----------------------------------------------------------------------------

void mobius::core_Timer::Restart ()
{
  m_fTimeStart = GetWallClockTime();
  m_fTimeCumul = 0.0;
  core_Chronometer::Restart();
}

//-----------------------------------------------------------------------------

void mobius::core_Timer::Show() const
{
  Show(std::cout);
}

//-----------------------------------------------------------------------------

double mobius::core_Timer::ElapsedTime() const
{
  if ( m_bIsStopped )
  {
    return m_fTimeCumul;
  }

  return m_fTimeCumul + GetWallClockTime() - m_fTimeStart;
}

//-----------------------------------------------------------------------------

void mobius::core_Timer::Show(double& theSeconds,
                              int&    theMinutes,
                              int&    theHours,
                              double& theCPUtime) const
{
  const double aTimeCumul = m_bIsStopped
                                 ? m_fTimeCumul
                                 : m_fTimeCumul + GetWallClockTime() - m_fTimeStart;
  timeToHoursMinutesSeconds (aTimeCumul, theHours, theMinutes, theSeconds);
  core_Chronometer::Show (theCPUtime);
}

//-----------------------------------------------------------------------------

void mobius::core_Timer::Show(std::ostream& out) const
{
  const double aTimeCumul = ElapsedTime();

  int anHours, aMinutes;
  double aSeconds;
  timeToHoursMinutesSeconds (aTimeCumul, anHours, aMinutes, aSeconds);

  std::streamsize prec = out.precision(12);
  out << "Elapsed time: " << anHours  << " Hours "   <<
                             aMinutes << " Minutes " <<
                             aSeconds << " Seconds\n";
  core_Chronometer::Show(out);
  out.precision(prec);
}

//-----------------------------------------------------------------------------

void mobius::core_Timer::Stop ()
{
  if ( !m_bIsStopped )
  {
    m_fTimeCumul += GetWallClockTime() - m_fTimeStart;
    core_Chronometer::Stop();
  }
}

//-----------------------------------------------------------------------------

void mobius::core_Timer::Start()
{
  if ( m_bIsStopped )
  {
    m_fTimeStart = GetWallClockTime();
    core_Chronometer::Start();
  }
}
