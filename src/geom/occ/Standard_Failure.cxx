// Copyright (c) 1998-1999 Matra Datavision
// Copyright (c) 1999-2014 OPEN CASCADE SAS
//
// This file is part of Open CASCADE Technology software library.
//
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with special exception defined in the file
// OCCT_LGPL_EXCEPTION.txt. Consult the file LICENSE_LGPL_21.txt included in OCCT
// distribution for complete text of the license and disclaimer of any warranty.
//
// Alternatively, this file may be used under the terms of Open CASCADE
// commercial license or contractual agreement.

#include <mobius/Standard_Failure.hxx>

#include <mobius/Standard_ErrorHandler.hxx>
#include <mobius/Standard_Macro.hxx>
#include <mobius/Standard_NoSuchObject.hxx>
#include <mobius/Standard_Type.hxx>

#include <string.h>

using namespace mobius::occ;

IMPLEMENT_STANDARD_RTTIEXT(Standard_Failure,Standard_Transient)

namespace
{
  //! Global parameter defining default length of stack trace.
  static Standard_Integer Standard_Failure_DefaultStackTraceLength = 0;
}

// =======================================================================
// function : StringRef::allocate_message
// purpose  :
// =======================================================================
Standard_Failure::StringRef* Standard_Failure::StringRef::allocate_message (const Standard_CString theString)
{
  if (theString == NULL
  || *theString == '\0')
  {
    return NULL;
  }

  const Standard_Size aLen = strlen (theString);
  StringRef* aStrPtr = (StringRef* )malloc (aLen + sizeof(Standard_Integer) + 1);
  if (aStrPtr != NULL)
  {
    strcpy ((char* )&aStrPtr->Message[0], theString);
    aStrPtr->Counter = 1;
  }
  return aStrPtr;
}

// =======================================================================
// function : StringRef::copy_message
// purpose  :
// =======================================================================
Standard_Failure::StringRef* Standard_Failure::StringRef::copy_message (Standard_Failure::StringRef* theString)
{
  if (theString == NULL)
  {
    return NULL;
  }

  ++theString->Counter;
  return theString;
}

// =======================================================================
// function : StringRef::deallocate_message
// purpose  :
// =======================================================================
void Standard_Failure::StringRef::deallocate_message (Standard_Failure::StringRef* theString)
{
  if (theString != NULL)
  {
    if (--theString->Counter == 0)
    {
      free ((void* )theString);
    }
  }
}

// =======================================================================
// function : Standard_Failure
// purpose  :
// =======================================================================
Standard_Failure::Standard_Failure()
: myMessage (NULL),
  myStackTrace (NULL)
{
}

// =======================================================================
// function : Standard_Failure
// purpose  :
// =======================================================================
Standard_Failure::Standard_Failure (const Standard_CString theDesc)
: myMessage (NULL),
  myStackTrace (NULL)
{
  myMessage = StringRef::allocate_message (theDesc);
}

// =======================================================================
// function : Standard_Failure
// purpose  :
// =======================================================================
Standard_Failure::Standard_Failure (const Standard_CString theDesc,
                                    const Standard_CString theStackTrace)
: myMessage (NULL),
  myStackTrace (NULL)
{
  myMessage = StringRef::allocate_message (theDesc);
  myStackTrace = StringRef::allocate_message (theStackTrace);
}

// =======================================================================
// function : Standard_Failure
// purpose  :
// =======================================================================
Standard_Failure::Standard_Failure (const Standard_Failure& theFailure)
: Standard_Transient (theFailure),
  myMessage (NULL),
  myStackTrace (NULL)
{
  myMessage    = StringRef::copy_message (theFailure.myMessage);
  myStackTrace = StringRef::copy_message (theFailure.myStackTrace);
}

// =======================================================================
// function : ~Standard_Failure
// purpose  :
// =======================================================================
Standard_Failure::~Standard_Failure()
{
  StringRef::deallocate_message (myMessage);
  StringRef::deallocate_message (myStackTrace);
}

// =======================================================================
// function : GetMessageString
// purpose  :
// =======================================================================
Standard_CString Standard_Failure::GetMessageString() const
{
  return myMessage != NULL
       ? myMessage->GetMessage()
       : "";
}

// =======================================================================
// function : SetMessageString
// purpose  :
// =======================================================================
void Standard_Failure::SetMessageString (const Standard_CString theDesc)
{
  if (theDesc == GetMessageString())
  {
    return;
  }

  StringRef::deallocate_message (myMessage);
  myMessage = StringRef::allocate_message (theDesc);
}

// =======================================================================
// function : GetStackString
// purpose  :
// =======================================================================
Standard_CString Standard_Failure::GetStackString() const
{
  return myStackTrace != NULL
       ? myStackTrace->GetMessage()
       : "";
}

// =======================================================================
// function : SetStackString
// purpose  :
// =======================================================================
void Standard_Failure::SetStackString (const Standard_CString theStack)
{
  if (theStack == GetStackString())
  {
    return;
  }

  StringRef::deallocate_message (myStackTrace);
  myStackTrace = StringRef::allocate_message (theStack);
}

// =======================================================================
// function : Raise
// purpose  :
// =======================================================================
void Standard_Failure::Raise (const Standard_CString theDesc)
{ 
  Handle(Standard_Failure) aFailure = new Standard_Failure();
  aFailure->Reraise (theDesc);
}

// =======================================================================
// function : Raise
// purpose  :
// =======================================================================
void Standard_Failure::Raise (const Standard_SStream& theReason)
{ 
  Handle(Standard_Failure) aFailure = new Standard_Failure();
  aFailure->Reraise (theReason);
}

// =======================================================================
// function : Reraise
// purpose  :
// =======================================================================
void Standard_Failure::Reraise (const Standard_CString theDesc)
{
  SetMessageString (theDesc);
  Reraise();
}

// =======================================================================
// function : Reraise
// purpose  :
// =======================================================================
void Standard_Failure::Reraise (const Standard_SStream& theReason)
{
  SetMessageString (theReason.str().c_str());
  Reraise();
}

// =======================================================================
// function : Reraise
// purpose  :
// =======================================================================
void Standard_Failure::Reraise()
{
  Throw();
}

// =======================================================================
// function : Jump
// purpose  :
// =======================================================================
void Standard_Failure::Jump()
{
#if defined (OCC_CONVERT_SIGNALS)
  Standard_ErrorHandler::Error (this);
  Standard_ErrorHandler::Abort (this);
#else
  Throw();
#endif
}

// =======================================================================
// function : Throw
// purpose  :
// =======================================================================
void Standard_Failure::Throw() const
{
  throw *this;
}

// =======================================================================
// function : Print
// purpose  :
// =======================================================================
void Standard_Failure::Print (Standard_OStream& theStream) const
{
  if (myMessage != NULL)
  {
    theStream << DynamicType() << ": " << GetMessageString();
  }
  else
  {
    theStream << DynamicType();
  }
  if (myStackTrace != NULL)
  {
    theStream << GetStackString();
  }
}

// =======================================================================
// function : NewInstance
// purpose  :
// =======================================================================
Handle(Standard_Failure) Standard_Failure::NewInstance (Standard_CString theString)
{
  return new Standard_Failure (theString);
}

// =======================================================================
// function : NewInstance
// purpose  :
// =======================================================================
Handle(Standard_Failure) Standard_Failure::NewInstance (Standard_CString theMessage,
                                                        Standard_CString theStackTrace)
{
  return new Standard_Failure (theMessage, theStackTrace);
}

// =======================================================================
// function : GetNbStackTraces
// purpose  :
// =======================================================================
Standard_Integer Standard_Failure::DefaultStackTraceLength()
{
  return Standard_Failure_DefaultStackTraceLength;
}

// =======================================================================
// function : SetNbStackTraces
// purpose  :
// =======================================================================
void Standard_Failure::SetDefaultStackTraceLength (Standard_Integer theNbStackTraces)
{
  Standard_Failure_DefaultStackTraceLength = theNbStackTraces;
}
