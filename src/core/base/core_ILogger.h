//-----------------------------------------------------------------------------
// Created on: July 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2018-present, OPEN CASCADE SAS
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
//    * Neither the name of OPEN CASCADE SAS nor the
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
//
// Web: http://dev.opencascade.org
//-----------------------------------------------------------------------------

#ifndef core_ILogger_HeaderFile
#define core_ILogger_HeaderFile

// Core includes
#include <mobius/core_excMsgInit.h>
#include <mobius/core_TimeStamp.h>
#include <mobius/core_Vars.h>

namespace mobius {

//-----------------------------------------------------------------------------

#define MobiusInfo(PriorityShort) \
  core_MsgStream(MsgSeverity_Information, MsgPriority_##PriorityShort)

#define MobiusNotice(PriorityShort) \
  core_MsgStream(MsgSeverity_Notice, MsgPriority_##PriorityShort)

#define MobiusWarn(PriorityShort) \
  core_MsgStream(MsgSeverity_Warning, MsgPriority_##PriorityShort)

#define MobiusErr(PriorityShort) \
  core_MsgStream(MsgSeverity_Error, MsgPriority_##PriorityShort)

//-----------------------------------------------------------------------------

//! \ingroup MOBIUS_CORE
//!
//! Type definition for logging arguments of heterogeneous types.
typedef std::vector< t_ptr<core_VarBase> > core_MsgArguments;

//-----------------------------------------------------------------------------

//! \ingroup MOBIUS_CORE
//!
//! Priority of logged message.
enum core_MsgPriority
{
  MsgPriority_Normal = 1, //!< Nothing special.
  MsgPriority_High,       //!< Important.
};

//-----------------------------------------------------------------------------

//! \ingroup MOBIUS_CORE
//!
//! Severity of logged message.
enum core_MsgSeverity
{
  MsgSeverity_Information = 1, //!< Just information message.
  MsgSeverity_Notice,          //!< Notice message (can be important).
  MsgSeverity_Warning,         //!< Warning message.
  MsgSeverity_Error            //!< Error message.
};

//-----------------------------------------------------------------------------

//! \ingroup MOBIUS_CORE
//!
//! Generic logging message.
struct core_Msg
{
  //! Priority tag.
  core_MsgPriority Priority;

  //! Severity tag.
  core_MsgSeverity Severity;

  //! Message text. It is ASCII string as we consider it to be the
  //! localization key.
  std::string MsgKey;

  //! Arguments for logging message.
  core_MsgArguments Arguments;

  //! Timestamp.
  t_ptr<core_TimeStamp> TimeStamp;

  //! Default ctor.
  core_Msg() : Priority(MsgPriority_Normal), Severity(MsgSeverity_Information) {}

  //! Complete ctor.
  //! \param[in] priority  message priority tag.
  //! \param[in] severity  message severity tag.
  //! \param[in] msgKey    message localization key.
  //! \param[in] arguments arguments for the logging message if any.
  //! \param[in] timeStamp timestamp.
  core_Msg(const core_MsgPriority       priority,
           const core_MsgSeverity       severity,
           const std::string&           msgKey,
           const core_MsgArguments&     arguments = core_MsgArguments(),
           const t_ptr<core_TimeStamp>& timeStamp = nullptr)
  : Priority  (priority),
    Severity  (severity),
    MsgKey    (msgKey),
    Arguments (arguments),
    TimeStamp (timeStamp)
  {}
};

//-----------------------------------------------------------------------------

//! \ingroup MOBIUS_CORE
//!
//! Interface for logging tools.
class core_ILogger : public core_OBJECT
{
public:

  mobiusCore_EXPORT virtual
    ~core_ILogger();

// Logging API:
public:

  virtual void
    SendInfo(const std::string&           msgKey,
             const core_MsgPriority       priority  = MsgPriority_Normal,
             const core_MsgArguments&     arguments = core_MsgArguments(),
             const t_ptr<core_TimeStamp>& timeStamp = nullptr) = 0;

  virtual void
    SendNotice(const std::string&           msgKey,
               const core_MsgPriority       priority  = MsgPriority_Normal,
               const core_MsgArguments&     arguments = core_MsgArguments(),
               const t_ptr<core_TimeStamp>& timeStamp = nullptr) = 0;

  virtual void
    SendWarning(const std::string&           msgKey,
                const core_MsgPriority       priority  = MsgPriority_Normal,
                const core_MsgArguments&     arguments = core_MsgArguments(),
                const t_ptr<core_TimeStamp>& timeStamp = nullptr) = 0;

  virtual void
    SendError(const std::string&           msgKey,
              const core_MsgPriority       priority  = MsgPriority_Normal,
              const core_MsgArguments&     arguments = core_MsgArguments(),
              const t_ptr<core_TimeStamp>& timeStamp = nullptr) = 0;

public:

  //! Iterates over the passed list of messages and sends them to the logger
  //! taking care of the severity/priority tags for each message.
  //! \param[in] messages list of messages to dispatch.
  virtual void Dispatch(const std::vector<core_Msg>& messages)
  {
    for ( size_t k = 0; k < messages.size(); ++k )
    {
      const core_Msg& msg = messages[k];

      switch ( msg.Severity )
      {
        case MsgSeverity_Information:
        {
          this->SendInfo(msg.MsgKey, msg.Priority, msg.Arguments, msg.TimeStamp);
          break;
        }
        case MsgSeverity_Notice:
        {
          this->SendNotice(msg.MsgKey, msg.Priority, msg.Arguments, msg.TimeStamp);
          break;
        }
        case MsgSeverity_Warning:
        {
          this->SendWarning(msg.MsgKey, msg.Priority, msg.Arguments, msg.TimeStamp);
          break;
        }
        case MsgSeverity_Error:
        {
          this->SendError(msg.MsgKey, msg.Priority, msg.Arguments, msg.TimeStamp);
          break;
        }
        default: break;
      }
    }
  }

};

//-----------------------------------------------------------------------------

//! \ingroup MOBIUS_CORE
//!
//! Convenience tool for message streaming.
class core_MsgStream
{
public:

  //! Default ctor.
  core_MsgStream()
  {
    m_severity          = MsgSeverity_Information;
    m_priority          = MsgPriority_Normal;
    m_bIsDummy          = true;
    m_bIsMsgInitialized = false;
  }

  //! Constructor.
  //! \param[in] severity severity of the Log Message.
  //! \param[in] priority priority of the Log Message.
  core_MsgStream(const core_MsgSeverity& severity,
                 const core_MsgPriority& priority)
  {
    m_severity          = severity;
    m_priority          = priority;
    m_bIsDummy          = false;
    m_bIsMsgInitialized = false;
  }

  //! Converter to Log Message.
  //! \return Log Message.
  operator core_Msg()
  {
    return core_Msg(m_priority, m_severity, m_msg, m_args, nullptr);
  }

  //! Pushes the passed string to the logging stream.
  //! \param[in] str string to stream.
  //! \return this instance for further streaming.
  core_MsgStream& operator<<(const char* str)
  {
    return this->operator<<( std::string(str) );
  }

  //! Pushes the passed string to the logging stream.
  //! \param[in] str string to stream.
  //! \return this instance for further streaming.
  core_MsgStream& operator<<(const std::string& str)
  {
    if ( m_bIsDummy )
      return *this;

    if ( !m_bIsMsgInitialized )
    {
      m_msg               = str;
      m_bIsMsgInitialized = true;
    }
    else
    {
      t_ptr<core_VarString> var = new core_VarString(str);
      m_args.push_back(var);
    }

    return *this;
  }

  //! Pushes the passed value to the logging stream.
  //! \param[in] val value to stream.
  //! \return this instance for further streaming.
  core_MsgStream& operator<<(const int val)
  {
    if ( m_bIsDummy )
      return *this;

    if ( !m_bIsMsgInitialized )
      throw core_excMsgInit();

    t_ptr<core_VarInt> var = new core_VarInt(val);
    m_args.push_back(var);

    return *this;
  }

  //! Pushes the passed value to the logging stream.
  //! \param[in] val value to stream.
  //! \return this instance for further streaming.
  core_MsgStream& operator<<(const size_t val)
  {
    if ( m_bIsDummy )
      return *this;

    if ( !m_bIsMsgInitialized )
      throw core_excMsgInit();

    t_ptr<core_VarInt> var = new core_VarInt( int(val) );
    m_args.push_back(var);

    return *this;
  }

  //! Pushes the passed value to the logging stream.
  //! \param[in] val value to stream.
  //! \return this instance for further streaming.
  core_MsgStream& operator<<(const double val)
  {
    if ( m_bIsDummy )
      return *this;

    if ( !m_bIsMsgInitialized )
      throw core_excMsgInit();

    t_ptr<core_VarReal> var = new core_VarReal(val);
    m_args.push_back(var);

    return *this;
  }

  //! Pushes the passed value to the logging stream.
  //! \param[in] val value to stream.
  //! \return this instance for further streaming.
  core_MsgStream& operator<<(const bool val)
  {
    return this->operator<<(val ? "true" : "false");
  }

  //! Pushes the streamed message to the passed Logger.
  //! \param[in] logger target Logger.
  void operator>>(const t_ptr<core_ILogger>& logger)
  {
    if ( m_bIsDummy )
      return;

    if ( logger.IsNull() )
      return;

    if ( m_severity == MsgSeverity_Information )
    {
      logger->SendInfo(m_msg, m_priority, m_args);
    }
    else if ( m_severity == MsgSeverity_Notice )
    {
      logger->SendNotice(m_msg, m_priority, m_args);
    }
    else if ( m_severity == MsgSeverity_Warning )
    {
      logger->SendWarning(m_msg, m_priority, m_args);
    }
    else if ( m_severity == MsgSeverity_Error )
    {
      logger->SendError(m_msg, m_priority, m_args);
    }
  }

  //! Accessor for severity.
  //! \return message severity.
  core_MsgSeverity GetSeverity() const
  {
    return m_severity;
  }

  //! Accessor for priority.
  //! \return message priority.
  core_MsgPriority GetPriority() const
  {
    return m_priority;
  }

  //! Accessor for text.
  //! \return message text.
  const std::string& GetText() const
  {
    return m_msg;
  }

  //! Accessor for arguments.
  //! \return message arguments.
  const core_MsgArguments& GetArgs() const
  {
    return m_args;
  }

private:

  //! Message priority.
  core_MsgPriority m_priority;

  //! Message severity.
  core_MsgSeverity m_severity;

  //! Logging message.
  std::string m_msg;

  //! Logging arguments.
  core_MsgArguments m_args;

  //! Internal status.
  bool m_bIsMsgInitialized;

  //! Indicates whether Logging Stream is dummy or not.
  bool m_bIsDummy;

};

}

#endif
