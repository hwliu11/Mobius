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

#ifndef test_ProgressNotifier_h
#define test_ProgressNotifier_h

// Test includes
#include <mobius/test.h>

// Core includes
#include <mobius/core_IProgressNotifier.h>

namespace mobius {

//! Notification tool to take care of algorithmic messages.
class test_ProgressNotifier : public core_IProgressNotifier
{
public:

  //! Constructor.
  //! \param[in,out] os output stream.
  test_ProgressNotifier(std::ostream& os);

public:

  //! Cleans up the internal state of the Progress Notifier, so that it
  //! becomes ready to track another job.
  virtual void
    Reset() override;

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
    Init(const int capacity = INT_MAX) override;

  //! Returns the capacity value.
  //! \return requested capacity value.
  virtual int
    GetCapacity() const override;

  //! Returns true if the capacity value is infinite.
  //! \return true/false.
  virtual bool
    IsInfinite() const override;

  //! Sets message (localization) key.
  //! \param[in] msgKey message key to set.
  virtual void
    SetMessageKey(const std::string& msgKey) override;

  //! Returns message localization key.
  //! \return localization key.
  virtual const std::string&
    GetMessageKey() const override;

  //! Sets the job status.
  //! \param[in] status progress status to set.
  virtual void
    SetProgressStatus(const core_ProgressStatus status) override;

  //! Returns current progress status.
  //! \return the ultimate progress status.
  virtual core_ProgressStatus
    GetProgressStatus() const override;

  //! Requests job cancellation.
  virtual void
    AskCancel() override;

  //! Checks whether the job is being canceled.
  //! \return true/false.
  virtual bool
    IsCancelling() override;

  //! Checks whether the job is in running state.
  //! \return true/false.
  virtual bool
    IsRunning() override;

  //! Checks whether the job is in failed state.
  //! \return true/false.
  virtual bool
    IsFailed() override;

  //! Returns the currently cumulated progress value.
  //! \return current cumulative progress.
  virtual int
    GetCurrentProgress() const override;

// Interface to be used by algorithms:
public:

  //! This method is used to increment the progress value by the passed step.
  //! \param[in] incr progress value to increment by.
  virtual void
    StepProgress(const int incr) override;

  //! This method is used to set the progress value.
  //! \param[in] progress progress value to set.
  virtual void
    SetProgress(const int progress) override;

  //! This method is used to send a logging message.
  //! \param[in] message   message string (normally it is i18n key).
  //! \param[in] severity  message severity (info, notice, warning, error).
  //! \param[in] priority  message priority (normal, high).
  //! \param[in] arguments message arguments (if any).
  virtual void
    SendLogMessage(const std::string&       message,
                   const core_MsgSeverity   severity,
                   const core_MsgPriority   priority  = MsgPriority_Normal,
                   const core_MsgArguments& arguments = core_MsgArguments()) override;

  //! This method is used to send a logging message in a stream form.
  //! \param[in] logStream logging stream.
  virtual void
    SendLogMessage(const core_MsgStream& logStream) override;

private:

  //! Output stream.
  std::ostream& m_out;

  //! Current progress.
  int m_iProgress;

  //! Maximum allowed capacity.
  int m_iCapacity;

  //! Message key for short-description.
  std::string m_msgKey;

  //! Progress status.
  core_ProgressStatus m_status;

  //! Service flag.
  bool m_SendLogMessageCalled;

private:

  void operator=(const test_ProgressNotifier&) = delete;
  test_ProgressNotifier(const test_ProgressNotifier&) = delete;

};

};

#endif
