//-----------------------------------------------------------------------------
// Created on: 07 December 2013
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

#ifndef core_TimeStamp_HeaderFile
#define core_TimeStamp_HeaderFile

// core includes
#include <mobius/core_OBJECT.h>
#include <mobius/core_Ptr.h>

// STD includes
#include <time.h>

namespace mobius {

//-----------------------------------------------------------------------------
// TimeStamp structure
//-----------------------------------------------------------------------------

//! Utility class representing general-purpose timestamp down to the
//! seconds. Actually this class extends time_t value with addendum
//! integer value required for uniqueness (this integer is meaningless,
//! it has nothing related to timing).
class core_TimeStamp : public core_OBJECT
{
public:

  //! Time.
  time_t Time;

  //! Additional integer for uniqueness.
  int Internal;

public:

  //! Default constructor.
  core_TimeStamp() : core_OBJECT(), Time(-1), Internal(0)
  {}

  //! Complete constructor.
  //! \param time [in] time to initialize the unique timestamp with.
  //! \param internalCount [in] additional integer value to provide
  //!        uniqueness of the generated timestamp.
  core_TimeStamp(const time_t time, const int internalCount)
  : core_OBJECT(), Time(time), Internal(internalCount)
  {}

  //! Destructor.
  ~core_TimeStamp()
  {}

  //! Checks whether this timestamp is nullptr (not initialized).
  //! \return true/false.
  bool IsOrigin()
  {
    return (Time == -1) && (Internal == 0);
  }

  //! Checks whether this timestamp is equal to the passed one.
  //! \param Other [in] another timestamp to compare with.
  //! \return true/false.
  bool IsEqual(const core_Ptr<core_TimeStamp>& Other) const
  {
    return (Time == Other->Time) && (Internal == Other->Internal);
  }

  //! Checks whether this timestamp is less than the passed one.
  //! \param Other [in] another timestamp to compare with.
  //! \return true/false.
  bool IsLess(const core_Ptr<core_TimeStamp>& Other) const
  {
    return (Time < Other->Time) || ( (Time == Other->Time) && (Internal < Other->Internal) );
  }

  //! Checks whether this timestamp is less or equal to the passed one.
  //! \param Other [in] another timestamp to compare with.
  //! \return true/false.
  bool LessOrEqual(const core_Ptr<core_TimeStamp>& Other) const
  {
    return this->IsLess(Other) || this->IsEqual(Other);
  }

  //! Checks whether this timestamp is greater than the passed one.
  //! \param Other [in] another timestamp to compare with.
  //! \return true/false.
  bool IsGreater(const core_Ptr<core_TimeStamp>& Other) const
  {
    return (Time > Other->Time) || ( (Time == Other->Time) && (Internal > Other->Internal) );
  }

  //! Checks whether this timestamp is greater or equal to the passed one.
  //! \param Other [in] timestamp to compare with.
  //! \return true/false.
  bool IsGreaterOrEqual(const core_Ptr<core_TimeStamp>& Other) const
  {
    return this->IsGreater(Other) || this->IsEqual(Other);
  }

  //! Copies the timestamp.
  //! \return copy of the timestamp.
  core_Ptr<core_TimeStamp> Clone() const
  {
    return new core_TimeStamp(Time, Internal);
  }

  //! Dumps this timestamp to the passed output stream.
  //! \param out [in/out] stream to dump the timestamp to.
  void Dump(std::ostream* out) const
  {
    *out << this->ToString().c_str();
  }

  //! Converts timestamp to string.
  //! \param useInternal [in] indicates whether the internal index must be
  //!        included into the string representation as well.
  //! \param isCompatible [in] indicates whether the requested string
  //!        representation has to contain "safe" characters only. In such
  //!        a form you can use it, for instance, as a filename.
  //! \return string representation of timestamp.
  std::string ToString(const bool useInternal = true,
                       const bool isCompatible = false) const
  {
    std::string res;

#ifdef _WIN32
    char buf[26];
    ctime_s(buf, 26, &Time);
    buf[24] = '\0'; // Replace EOL [\n\0 --> \0\0]

    res += buf;
    if ( useInternal )
      res += ":: [" + core::str::to_string(Internal) + "]";

    if ( isCompatible )
    {
      core::str::replace_all(res, ":", "");
      core::str::replace_all(res, "[", "");
      core::str::replace_all(res, "]", "");
      core::str::replace_all(res, " ", "_");
    }
#else
    std::stringstream ss;
    ss << Time;
    res = ss.str();
#endif

    return res;
  }
};

//-----------------------------------------------------------------------------
// TimeStampTool utility
//-----------------------------------------------------------------------------

//! Auxiliary class generating unique and monotonic timestamps.
//! This class operates with standard time_t type not attempting to cast it
//! to any primitive (like integer, long etc). As this timestamp allows
//! serialization, it needs to be represented somehow with simple integer
//! basis. Even though straightforward conversion from time_t to int does
//! the trick, it is not really a solution as it leads to injection of Y2K38
//! problem into the application domain. The natural alternative solution is
//! to store standard "tm" structure containing only integer values and
//! being easily serialized this way.
//!
//! Another peculiarity here is that time_t value does not contain milliseconds
//! and cannot provide enough timing granularity for some cases. This is
//! resolved by extending that type with additional atomically incremented
//! static integer value. This value is used to maintain uniqueness only and
//! does not have any wise meaning.
class core_TimeStampTool
{
public:

  mobiusCore_EXPORT static core_Ptr<core_TimeStamp>
    Generate();

  mobiusCore_EXPORT static std::vector<int>
    AsChunked(const core_Ptr<core_TimeStamp>& TS);

  mobiusCore_EXPORT static core_Ptr<core_TimeStamp>
    FromChunked(const std::vector<int>& chunked);

};

}

#endif
