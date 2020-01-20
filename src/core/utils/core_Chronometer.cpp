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

// Own include
#include <mobius/core_Chronometer.h>

#ifndef _WIN32

#include <sys/times.h>
#include <unistd.h>

#ifdef SOLARIS
  #include <sys/resource.h>
#endif

#ifndef sysconf
  #define _sysconf sysconf
#endif

#if defined(DECOSF1)
  #include <time.h>
#endif

#ifndef CLK_TCK
  #define CLK_TCK CLOCKS_PER_SEC
#endif

#if (defined(__APPLE__))
  #include <mach/task.h>
  #include <mach/mach.h>
#endif

//=======================================================================
//function : GetProcessCPU
//purpose  :
//=======================================================================
void mobius::core_Chronometer::GetProcessCPU(double& theUserSeconds,
                                             double& theSystemSeconds)
{
#if defined(__linux__) || defined(__FreeBSD__) || defined(__ANDROID__) || defined(__QNX__)
  static const long aCLK_TCK = sysconf(_SC_CLK_TCK);
#else
  static const long aCLK_TCK = CLK_TCK;
#endif

  tms aCurrentTMS;
  times (&aCurrentTMS);

  theUserSeconds   = (double)aCurrentTMS.tms_utime / aCLK_TCK;
  theSystemSeconds = (double)aCurrentTMS.tms_stime / aCLK_TCK;
}

//=======================================================================
//function : GetThreadCPU
//purpose  :
//=======================================================================
void mobius::core_Chronometer::GetThreadCPU(double& theUserSeconds,
                                            double& theSystemSeconds)
{
  theUserSeconds = theSystemSeconds = 0.0;
#if (defined(__APPLE__))
  struct task_thread_times_info aTaskInfo;
  mach_msg_type_number_t aTaskInfoCount = TASK_THREAD_TIMES_INFO_COUNT;
  if (KERN_SUCCESS == task_info(mach_task_self(), TASK_THREAD_TIMES_INFO,
      (task_info_t )&aTaskInfo, &aTaskInfoCount))
  {
    theUserSeconds   = double(aTaskInfo.user_time.seconds)   + 0.000001 * aTaskInfo.user_time.microseconds;
    theSystemSeconds = double(aTaskInfo.system_time.seconds) + 0.000001 * aTaskInfo.system_time.microseconds;
  }
#elif (defined(_POSIX_TIMERS) && defined(_POSIX_THREAD_CPUTIME)) || defined(__ANDROID__) || defined(__QNX__)
  // on Linux, only user times are available for threads via clock_gettime()
  struct timespec t;
  if (!clock_gettime (CLOCK_THREAD_CPUTIME_ID, &t))
  {
    theUserSeconds = t.tv_sec + 0.000000001 * t.tv_nsec;
  }
#elif defined(SOLARIS)
  // on Solaris, both user and system times are available as LWP times
  struct rusage rut;
  if (!getrusage (RUSAGE_LWP, &rut))
  {
    theUserSeconds   = rut.ru_utime.tv_sec + 0.000001 * rut.ru_utime.tv_usec;
    theSystemSeconds = rut.ru_stime.tv_sec + 0.000001 * rut.ru_stime.tv_usec;
  }
#else
  #pragma error "OS is not supported yet; code to be ported"
#endif
}

#else

#define NOMINMAX
#include <windows.h>

//-----------------------------------------------------------------------------
// Function : EncodeFILETIME
// Purpose  : Encode time defined by FILETIME structure
//            (100s nanoseconds since January 1, 1601) to 64-bit integer
//-----------------------------------------------------------------------------

static inline __int64 EncodeFILETIME (PFILETIME pFt)
{
  __int64 qw;

  qw   = pFt -> dwHighDateTime;
  qw <<= 32;
  qw  |= pFt -> dwLowDateTime;

  return qw;
}

//-----------------------------------------------------------------------------

void mobius::core_Chronometer::GetProcessCPU(double& theUserSeconds,
                                             double& theSystemSeconds)
{
#ifndef OCCT_UWP
  FILETIME ftStart, ftExit, ftKernel, ftUser;
  ::GetProcessTimes (GetCurrentProcess(), &ftStart, &ftExit, &ftKernel, &ftUser);
  theUserSeconds   = 0.0000001 * EncodeFILETIME (&ftUser);
  theSystemSeconds = 0.0000001 * EncodeFILETIME (&ftKernel);
#else
  theUserSeconds = 0.0;
  theSystemSeconds = 0.0;
#endif
}

//-----------------------------------------------------------------------------

void mobius::core_Chronometer::GetThreadCPU(double& theUserSeconds,
                                            double& theSystemSeconds)
{
#ifndef OCCT_UWP
  FILETIME ftStart, ftExit, ftKernel, ftUser;
  ::GetThreadTimes (GetCurrentThread(), &ftStart, &ftExit, &ftKernel, &ftUser);
  theUserSeconds   = 0.0000001 * EncodeFILETIME (&ftUser);
  theSystemSeconds = 0.0000001 * EncodeFILETIME (&ftKernel);
#else
  theUserSeconds = 0.0;
  theSystemSeconds = 0.0;
#endif
}

#endif /* _WIN32 */

//-----------------------------------------------------------------------------

mobius::core_Chronometer::core_Chronometer(bool theThisThreadOnly)
: m_fStartCpuUser (0.0),
  m_fStartCpuSys  (0.0),
  m_fCumulCpuUser (0.0),
  m_fCumulCpuSys  (0.0),
  m_bIsStopped    (true),
  m_bIsThreadOnly (theThisThreadOnly)
{}

//-----------------------------------------------------------------------------

mobius::core_Chronometer::~core_Chronometer()
{}

//-----------------------------------------------------------------------------

void mobius::core_Chronometer::Reset()
{
  m_bIsStopped    = true;
  m_fStartCpuUser = m_fStartCpuSys = 0.;
  m_fCumulCpuUser = m_fCumulCpuSys = 0.;
}

//-----------------------------------------------------------------------------

void mobius::core_Chronometer::Restart()
{
  Reset();
  Start();
}

//-----------------------------------------------------------------------------

void mobius::core_Chronometer::Stop()
{
  if ( !m_bIsStopped )
  {
    double Curr_user, Curr_sys;
    if ( m_bIsThreadOnly )
      GetThreadCPU (Curr_user, Curr_sys);
    else
      GetProcessCPU (Curr_user, Curr_sys);

    m_fCumulCpuUser += Curr_user - m_fStartCpuUser;
    m_fCumulCpuSys  += Curr_sys  - m_fStartCpuSys;

    m_bIsStopped = true;
  }
}

//-----------------------------------------------------------------------------

void mobius::core_Chronometer::Start ()
{
  if ( m_bIsStopped )
  {
    if ( m_bIsThreadOnly )
      GetThreadCPU(m_fStartCpuUser, m_fStartCpuSys);
    else
      GetProcessCPU(m_fStartCpuUser, m_fStartCpuSys);

    m_bIsStopped = false;
  }
}

//-----------------------------------------------------------------------------

void mobius::core_Chronometer::Show() const
{
  Show (std::cout);
}

//-----------------------------------------------------------------------------

void mobius::core_Chronometer::Show(std::ostream& out) const
{
  double aCumulUserSec = 0.0, aCumulSysSec = 0.0;
  Show (aCumulUserSec, aCumulSysSec);
  std::streamsize prec = out.precision (12);
  out << "CPU user time: "   << aCumulUserSec << " seconds\n";
  out << "CPU system time: " << aCumulSysSec  << " seconds\n";
  out.precision (prec);
}

//-----------------------------------------------------------------------------

void mobius::core_Chronometer::Show(double& userSec,
                                    double& systemSec) const
{
  userSec   = m_fCumulCpuUser;
  systemSec = m_fCumulCpuSys;

  if ( m_bIsStopped )
  {
    return;
  }

  double aCurrUser, aCurrSys;
  if ( m_bIsThreadOnly )
    GetThreadCPU  (aCurrUser, aCurrSys);
  else
    GetProcessCPU (aCurrUser, aCurrSys);

  userSec   += aCurrUser - m_fStartCpuUser;
  systemSec += aCurrSys  - m_fStartCpuSys;
}
