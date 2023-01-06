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
#include <mobius/test_ProgressNotifier.h>

//-----------------------------------------------------------------------------

mobius::test_ProgressNotifier::test_ProgressNotifier(std::ostream& os)
: core_IProgressNotifier (),
  m_out                  (os),
  m_SendLogMessageCalled (true)
{
  this->Reset();
}

//-----------------------------------------------------------------------------

void mobius::test_ProgressNotifier::Reset()
{
  m_status               = Progress_Undefined;
  m_iCapacity            = 0;
  m_msgKey               = "";
  m_SendLogMessageCalled = true;
  m_iProgress            = 0;
}

//-----------------------------------------------------------------------------

void mobius::test_ProgressNotifier::Init(const int capacity)
{
  m_status    = Progress_Running;
  m_iCapacity = capacity;
  m_iProgress = 0;
}

//-----------------------------------------------------------------------------

int mobius::test_ProgressNotifier::GetCapacity() const
{
  return m_iCapacity;
}

//-----------------------------------------------------------------------------

bool mobius::test_ProgressNotifier::IsInfinite() const
{
  return m_iCapacity == INT_MAX;
}

//-----------------------------------------------------------------------------

void mobius::test_ProgressNotifier::SetMessageKey(const std::string& msgKey)
{
  m_msgKey = msgKey;
}

//-----------------------------------------------------------------------------

std::string mobius::test_ProgressNotifier::GetMessageKey() const
{
  return m_msgKey;
}

//-----------------------------------------------------------------------------

void mobius::test_ProgressNotifier::SetProgressStatus(const core_ProgressStatus status)
{
  m_status = status;

  if ( m_status >= Progress_Succeeded )
  {
    std::string prefix, verdict;
    //
    if ( m_status == Progress_Succeeded )
    {
      prefix = " + ";
      verdict = "succeeded";
    }
    else if ( m_status == Progress_Failed )
    {
      prefix = " - ";
      verdict = "failed";
    }
    else
    {
      prefix = " ? ";
      verdict = "done";
    }

    m_out << prefix << "   Operation " << verdict << ".\n";
  }
}

//-----------------------------------------------------------------------------

mobius::core_ProgressStatus
  mobius::test_ProgressNotifier::GetProgressStatus() const
{
  return m_status;
}

//-----------------------------------------------------------------------------

void mobius::test_ProgressNotifier::AskCancel()
{
  // Do nothing, no way to cancel in Draw console
}

//-----------------------------------------------------------------------------

bool mobius::test_ProgressNotifier::IsCancelling()
{
  return false;
}

//-----------------------------------------------------------------------------

bool mobius::test_ProgressNotifier::IsRunning()
{
  return (m_status == Progress_Running);
}

//-----------------------------------------------------------------------------

bool mobius::test_ProgressNotifier::IsFailed()
{
  return (m_status == Progress_Failed);
}

//-----------------------------------------------------------------------------

int mobius::test_ProgressNotifier::GetCurrentProgress() const
{
  return m_iProgress;
}

//-----------------------------------------------------------------------------

void mobius::test_ProgressNotifier::StepProgress(const int stepProgress)
{
  m_iProgress += stepProgress;
  m_SendLogMessageCalled = false;

  // Output current status (percentage or number of completed steps)
  int status;
  if ( this->IsInfinite() )
    status = this->GetCurrentProgress();
  else
    status = (int) ( (double) this->GetCurrentProgress() / this->GetCapacity() * 100);
  //
  m_out << "...\t progress"
        << (this->IsInfinite() ? " (infinite), current step is: " : ": ")
        << status
        << (this->IsInfinite() ? "" : "%")
        << "   "
        << "\r";
  //
  m_out.flush();
}

//-----------------------------------------------------------------------------

void mobius::test_ProgressNotifier::SetProgress(const int progress)
{
  m_iProgress = progress;
}

//-----------------------------------------------------------------------------

template<typename T, typename VT>
std::string toString(const mobius::t_ptr<mobius::core_VarBase>& varBase)
{
  mobius::t_ptr<T> var = mobius::t_ptr<T>::DownCast(varBase);
  //
  if ( var.IsNull() )
    return "";

  return mobius::core::str::to_string<VT>(var->Value);
}

//-----------------------------------------------------------------------------

std::string getString(const mobius::t_ptr<mobius::core_VarBase>& varBase)
{
  std::string varInt = toString<mobius::core_VarInt, int>(varBase);
  //
  if ( !varInt.empty() )
    return varInt;

  std::string varReal = toString<mobius::core_VarReal, double>(varBase);
  //
  if ( !varReal.empty() )
    return varReal;

  std::string varString = toString<mobius::core_VarString, std::string>(varBase);
  //
  if ( !varString.empty() )
    return varString;

  return "<empty arg>";
}

//-----------------------------------------------------------------------------

void mobius::test_ProgressNotifier::SendLogMessage(const std::string&       message,
                                                   const core_MsgSeverity   severity,
                                                   const core_MsgPriority   priority,
                                                   const core_MsgArguments& arguments)
{
  test_NotUsed(priority);

  std::string formatted = message;

  for ( int i = 0; i < arguments.size(); ++i )
  {
    std::string iarg = "%";
    iarg += core::str::to_string<int>(i + 1);

    size_t      posArg = formatted.find(iarg, 0);
    std::string sarg   = getString(arguments[i]);

    if ( posArg != std::string::npos )
    {
      core::str::replace_all(formatted, iarg, sarg);
    }
    else
    {
      formatted += " ";
      formatted += sarg;
    }
  }

  if ( !m_SendLogMessageCalled )
  {
    m_SendLogMessageCalled = true;
  }

  // Since carriage is returned in progress reporting by StepProgress() method,
  // we try to reserve enough characters to erase visually the line occupied
  // by the progress indication.
  if ( formatted.length() < 100 )
    for ( size_t k = formatted.length(); k < 100; ++k )
      formatted += " ";

  switch ( severity )
  {
    case MsgSeverity_Information:
      m_out << "...\t info: " << formatted << "\n";
      break;
    case MsgSeverity_Notice:
      m_out << "...\t notice: " << formatted << "\n";
      break;
    case MsgSeverity_Warning:
      m_out << "...\t warning: " << formatted << "\n";
      break;
    case MsgSeverity_Error:
      m_out << "...\t error: " << formatted << "\n";
      break;
    default:
      break;
  }

  m_out.flush();
}

//-----------------------------------------------------------------------------

void mobius::test_ProgressNotifier::SendLogMessage(const core_MsgStream& logStream)
{
  this->SendLogMessage( logStream.GetText(),
                        logStream.GetSeverity(),
                        logStream.GetPriority(),
                        logStream.GetArgs() );
}
