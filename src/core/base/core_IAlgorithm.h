//-----------------------------------------------------------------------------
// Created on: July 2018
//-----------------------------------------------------------------------------
// Copyright (c) 2017-present, Sergey Slyadnev
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

#ifndef core_IAlgorithm_HeaderFile
#define core_IAlgorithm_HeaderFile

// Core includes
#include <mobius/core_IPlotter.h>
#include <mobius/core_IProgressNotifier.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Interface for algorithms.
class core_IAlgorithm : public core_OBJECT
{
public:

  core_ProgressEntry& GetProgress() const { return m_progress; }
  core_PlotterEntry&  GetPlotter()  const { return m_plotter; }

protected:

  mobiusCore_EXPORT
    core_IAlgorithm(core_ProgressEntry progress,
                    core_PlotterEntry  plotter);

public:

  //! Sets status code as an integer.
  //! \param[in] status code to set.
  void SetStatusCode(const int status)
  {
    m_iStatusCode = status;
  }

  //! \return integer status code.
  int GetStatusCode() const
  {
    return m_iStatusCode;
  }

  //! Adds status to the currently stored one. The derived classes take
  //! responsibility to implement status codes as bitmasks like 0x01, 0x02,
  //! 0x04, 0x08, 0x10, 0x20, 0x40, etc. This may we can store several statuses
  //! in one integer variable.
  //! \param[in] statBit status bit to add to the current status.
  void AddStatusCode(const int statBit)
  {
    m_iStatusCode |= statBit;
  }

  //! Checks whether the stored status code contains bits for the passed
  //! status.
  //! \param[in] statBit bits to check.
  //! \return true/false.
  bool HasStatusCode(const int statBit) const
  {
    return (m_iStatusCode & statBit) > 0;
  }

protected:

  mutable core_ProgressEntry m_progress; //!< Progress Notifier.
  mutable core_PlotterEntry  m_plotter;  //!< Imperative Plotter.

  //! Status code which can be an error code, warning code or any other
  //! status which gives more detalisation on algorithm's execution state.
  int m_iStatusCode;

protected:

  core_IAlgorithm() : core_OBJECT() {} //!< Default ctor.

};

};

#endif
