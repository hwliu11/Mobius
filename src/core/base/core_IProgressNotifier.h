//-----------------------------------------------------------------------------
// Created on: July 2018
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

#ifndef core_IProgressNotifier_HeaderFile
#define core_IProgressNotifier_HeaderFile

// Core includes
#include <mobius/core_ILogger.h>

namespace mobius {

//-----------------------------------------------------------------------------

//! \ingroup AD_API
//!
//! Progress status.
enum core_ProgressStatus
{
  Progress_Undefined = 0, //!< No status defined, no job has been ever started.
  Progress_Running,       //!< Job is currently running.
  Progress_Succeeded,     //!< Job has been performed successfully.
  Progress_Failed,        //!< Job has been failed.
  Progress_Cancelled      //!< Job has been requested for cancellation.
};

//-----------------------------------------------------------------------------
// Progress Notifier
//-----------------------------------------------------------------------------

//! \ingroup MOBIUS_CORE
//!
//! Interface for Progress Notifier concept. Progress Notifier provides
//! messaging mechanism for communications between algorithmic and GUI layers.
//! It is normally used in cases when GUI thread is running separately from
//! the working one, however, it is also suitable in a single-threaded
//! context (actually, it depends on the used implementation). Progress
//! Notifier provides the following services:
//!
//! - Accumulate PROGRESS as a single integer value less or equal to CAPACITY.
//!
//! - Set progress message describing the currently performed
//!   job. This is normally an ASCII string localization key.
//!
//! - Set completeness state for the entire process. The following states
//!   are supported: Not Defined, Running, Succeeded, Failed, Cancelled.
class core_IProgressNotifier : public core_OBJECT
{
public:

  //! Cleans up the internal state of the Progress Notifier, so that it
  //! becomes ready to track another job.
  virtual void
    Reset() = 0;

  //! Initializes the Progress Notifier with the deterministic capacity
  //! value. Capacity represents the unitless overall progress value which
  //! can be ever collected by all running tasks.
  //!
  //! Please note, that by default the progress scale is declared with
  //! infinite capacity. Practically, it means that algorithm is not able
  //! to foresee the number of steps it will need to complete. Make sure that
  //! in such a case your interface reacts adequately (e.g. no percentage is
  //! shown to the user).
  //!
  //! \param[in] capacity capacity score to set (infinite by default).
  virtual void
    Init(const int capacity = INT_MAX) = 0;

  //! Returns the capacity value.
  //! \return requested capacity value.
  virtual int
    GetCapacity() const = 0;

  //! Returns true if the capacity value is infinite.
  //! \return true/false.
  virtual bool
    IsInfinite() const = 0;

  //! Sets message (localization) key.
  //! \param[in] msgKey message key to set.
  virtual void
    SetMessageKey(const std::string& msgKey) = 0;

  //! Returns message localization key.
  //! \return localization key.
  virtual const std::string&
    GetMessageKey() const = 0;

  //! Sets the job status.
  //! \param[in] status progress status to set.
  virtual void
    SetProgressStatus(const core_ProgressStatus status) = 0;

  //! Returns current progress status.
  //! \return the ultimate progress status.
  virtual core_ProgressStatus
    GetProgressStatus() const = 0;

  //! Requests job cancellation.
  virtual void
    AskCancel() = 0;

  //! Checks whether the job is being canceled.
  //! \return true/false.
  virtual bool
    IsCancelling() = 0;

  //! Checks whether the job is in running state.
  //! \return true/false.
  virtual bool
    IsRunning() = 0;

  //! Checks whether the job is in failed state.
  //! \return true/false.
  virtual bool
    IsFailed() = 0;

  //! Returns the currently cumulated progress value.
  //! \return current cumulative progress.
  virtual int
    GetCurrentProgress() const = 0;

// Interface to be used by algorithms:
public:

  //! This method is used to increment the progress value by the passed step.
  //! \param[in] incr progress value to increment by.
  virtual void
    StepProgress(const int incr) = 0;

  //! This method is used to set the progress value.
  //! \param[in] progress progress value to set.
  virtual void
    SetProgress(const int progress) = 0;

  //! This method is used to send a logging message.
  //! \param[in] message   message string (normally it is i18n key).
  //! \param[in] severity  message severity (info, notice, warning, error).
  //! \param[in] priority  message priority (normal, high).
  //! \param[in] arguments message arguments (if any).
  virtual void
    SendLogMessage(const std::string&       message,
                   const core_MsgSeverity   severity,
                   const core_MsgPriority   priority  = MsgPriority_Normal,
                   const core_MsgArguments& arguments = core_MsgArguments()) = 0;

  //! This method is used to send a logging message in a stream form.
  //! \param[in] logStream logging stream.
  virtual void
    SendLogMessage(const core_MsgStream& logStream) = 0;

};

//-----------------------------------------------------------------------------
// Progress Entry
//-----------------------------------------------------------------------------

//! \ingroup MOBIUS_CORE
//!
//! Convenient way to work with Progress Notifier. This class is mostly
//! useful due to its NULL-safe approach to working with the underlying
//! Progress Notifier.
class core_ProgressEntry
{
public:

  //! Default ctor.
  core_ProgressEntry() {}

  //! Dummy conversion constructor.
  core_ProgressEntry(int) {}

  //! Copy ctor.
  //! \param[in] entry progress entry to copy.
  core_ProgressEntry(const core_ProgressEntry& entry)
  {
    m_PNotifier = entry.m_PNotifier;
  }

  //! Complete ctor.
  //! \param[in] progress Progress Notifier instance to set.
  core_ProgressEntry(const t_ptr<core_IProgressNotifier>& progress)
  {
    m_PNotifier = progress;
  }

public:

  //! Null-safe Reset method for Progress Notifier.
  void Reset()
  {
    if ( !m_PNotifier.IsNull() )
      m_PNotifier->Reset();
  }

  //! Null-safe Init() method for Progress Notifier.
  //! \param[in] capacity capacity to set.
  void Init(const int capacity = INT_MAX)
  {
    if ( !m_PNotifier.IsNull() )
      m_PNotifier->Init(capacity);
  }

  //! Null-safe accessor for the capacity value.
  //! \return requested capacity value.
  int GetCapacity() const
  {
    if ( !m_PNotifier.IsNull() )
      return m_PNotifier->GetCapacity();

    return 0;
  }

  //! Null-safe checker for infinite capacity.
  //! \return true/false.
  bool IsInfinite() const
  {
    if ( !m_PNotifier.IsNull() )
      return m_PNotifier->IsInfinite();

    return false;
  }

  //! Null-safe SetMessageKey() method for Progress Notifier.
  //! \param[in] msgKey localization key to set.
  void SetMessageKey(const std::string& msgKey)
  {
    if ( !m_PNotifier.IsNull() )
      m_PNotifier->SetMessageKey(msgKey);
  }

  //! Null-safe accessor for the message localization key.
  //! \return localization key.
  std::string GetMessageKey() const
  {
    if ( !m_PNotifier.IsNull() )
      return m_PNotifier->GetMessageKey();

    return std::string();
  }

  //! Null-safe SetProgressStatus() method for Progress Notifier.
  //! \param[in] status progress status to set.
  void SetProgressStatus(const core_ProgressStatus status)
  {
    if ( !m_PNotifier.IsNull() )
      m_PNotifier->SetProgressStatus(status);
  }

  //! Null-safe accessor for the current progress status.
  //! \return the ultimately set progress status.
  core_ProgressStatus GetProgressStatus() const
  {
    if ( !m_PNotifier.IsNull() )
      return m_PNotifier->GetProgressStatus();

    return Progress_Undefined;
  }

  //! Null-safe AskCancel() method for Progress Notifier.
  void AskCancel()
  {
    if ( !m_PNotifier.IsNull() )
      m_PNotifier->AskCancel();
  }

  //! Null-safe IsCancelling() checker.
  //! \return true/false.
  bool IsCancelling() const
  {
    if ( !m_PNotifier.IsNull() )
      return m_PNotifier->IsCancelling();

    return false;
  }

  //! Null-safe IsRunning() checker.
  //! \return true/false.
  bool IsRunning() const
  {
    if ( !m_PNotifier.IsNull() )
      return m_PNotifier->IsRunning();

    return false;
  }

  //! Null-safe IsFailed() checker.
  //! \return true/false.
  bool IsFailed()
  {
    if ( !m_PNotifier.IsNull() )
      return m_PNotifier->IsFailed();

    return false;
  }

  //! Null-safe accessor for the current progress.
  //! \return current progress.
  int GetCurrentProgress() const
  {
    if ( !m_PNotifier.IsNull() )
      return m_PNotifier->GetCurrentProgress();

    return 0;
  }

  //! Accessor for the underlying Progress Notifier.
  //! \return Progress Notifier instance.
  const t_ptr<core_IProgressNotifier>& Access() const
  {
    return m_PNotifier;
  }

// Thread-safe methods:
public:

  //! Null-safe StepProgress() method for Progress Notifier.
  //! \param[in] incr value to increment the currently accumulated progress.
  void StepProgress(const int incr) const
  {
    if ( !m_PNotifier.IsNull() )
      m_PNotifier->StepProgress(incr);
  }

  //! Null-safe SetProgress() method for Progress Notifier.
  //! \param[in] value new progress score to set.
  void SetProgress(const int value) const
  {
    if ( !m_PNotifier.IsNull() )
      m_PNotifier->SetProgress(value);
  }

  //! Null-safe SendLogMessage() method for Progress Notifier.
  //! \param[in] message   message string (normally it is i18n key).
  //! \param[in] severity  message severity (info, notice, warning, error).
  //! \param[in] priority  message priority (normal, high).
  //! \param[in] arguments message arguments (if any).
  void SendLogMessage(const std::string&       message,
                      const core_MsgSeverity   severity,
                      const core_MsgPriority   priority  = MsgPriority_Normal,
                      const core_MsgArguments& arguments = core_MsgArguments()) const
  {
    if ( !m_PNotifier.IsNull() )
      m_PNotifier->SendLogMessage(message, severity, priority, arguments);
  }

  //! Null-safe SendLogMessage() method for Progress Notifier.
  //! \param[in] logStream logging stream.
  void SendLogMessage(const core_MsgStream& logStream) const
  {
    if ( !m_PNotifier.IsNull() )
      m_PNotifier->SendLogMessage(logStream);
  }

private:

  //! Managed instance of Progress Notifier.
  t_ptr<core_IProgressNotifier> m_PNotifier;

};

};

#endif
