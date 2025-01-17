// Copyright (c) 1998-1999 Matra Datavision
// Copyright (c) 1999-2013 OPEN CASCADE SAS
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

#ifndef Standard_Integer_HeaderFile
#define Standard_Integer_HeaderFile

#include <mobius/Standard_Std.hxx>
#include <mobius/Standard_TypeDef.hxx>
#include <mobius/Standard_Values.hxx>

namespace mobius {
namespace occ {

// ===============
// Inline methods
// ===============

// ------------------------------------------------------------------
// Abs : Returns the absolute value of an Integer
// ------------------------------------------------------------------
inline  int Abs (const int Value)
{
  return Value >= 0 ? Value : -Value;
}

// ------------------------------------------------------------------
// IsEven : Returns Standard_True if an integer is even
// ------------------------------------------------------------------
inline Standard_Boolean IsEven (const int Value)
{ return Value % 2 == 0; }


// ------------------------------------------------------------------
// IsOdd : Returns Standard_True if an integer is odd
// ------------------------------------------------------------------
inline Standard_Boolean IsOdd (const int Value)
{ return Value % 2 == 1; }

// ------------------------------------------------------------------
// Max : Returns the maximum integer between two integers
// ------------------------------------------------------------------
inline int  Max (const int Val1,
			      const int Val2)
{
  return Val1 >= Val2 ? Val1 : Val2;
}

// ------------------------------------------------------------------
// Min : Returns the minimum integer between two integers
// ------------------------------------------------------------------
inline int  Min (const int Val1,
			      const int Val2)
{
  return Val1 <= Val2 ? Val1 : Val2;
}

// ------------------------------------------------------------------
// Modulus : Returns the remainder of division between two integers
// ------------------------------------------------------------------
inline int  Modulus (const int Value,
				  const int Divisor)
{ return Value % Divisor; }

// ------------------------------------------------------------------
// Square : Returns the square of an integer
// ------------------------------------------------------------------
inline int Square(const int Value)
{ return Value * Value; }

// ------------------------------------------------------------------
// IntegerFirst : Returns the minimum value of an integer
// ------------------------------------------------------------------
inline int  IntegerFirst()
{ return INT_MIN; }

// ------------------------------------------------------------------
// IntegerLast : Returns the maximum value of an integer
// ------------------------------------------------------------------
inline int  IntegerLast()
{ return INT_MAX; }

// ------------------------------------------------------------------
// IntegerSize : Returns the size in digits of an integer
// ------------------------------------------------------------------
inline int  IntegerSize()
{ return BITS(int); }


//! Computes a hash code for the given value of some integer type, in range [1, theUpperBound]
//! @tparam TheInteger the type of the integer which hash code is to be computed
//! @param theValue the value of the TheInteger type which hash code is to be computed
//! @param theMask the mask for the last bits of the value that are used in the computation of a hash code
//! @param theUpperBound the upper bound of the range a computing hash code must be within
//! @return a computed hash code, in range [1, theUpperBound]
template <typename TheInteger>
typename opencascade::std::enable_if<opencascade::is_integer<TheInteger>::value, int>::type
IntegerHashCode (const TheInteger                                                theValue,
                 const typename opencascade::disable_deduction<TheInteger>::type theMask,
                 const int                                          theUpperBound)
{
  return static_cast<int> ((theValue & theMask) % theUpperBound + 1);
}

//! Computes a hash code for the given value of the int type, in range [1, theUpperBound]
//! @param theValue the value of the int type which hash code is to be computed
//! @param theUpperBound the upper bound of the range a computing hash code must be within
//! @return a computed hash code, in range [1, theUpperBound]
inline int HashCode (const int theValue,
                                  const int theUpperBound)
{
  // return (Abs (theMe) % theUpper) + 1;
  return IntegerHashCode(theValue, IntegerLast(), theUpperBound);
}

// ------------------------------------------------------------------
// IsEqual : Returns Standard_True if two integers are equal
// ------------------------------------------------------------------
inline Standard_Boolean IsEqual (const int theOne,
                                 const int theTwo)
{
  return theOne == theTwo;
}

//! Computes a hash value for the given unsigned integer, in range [1, theUpperBound]
//! @param theValue the unsigned integer which hash code is to be computed
//! @param theUpperBound the upper bound of the range a computing hash code must be within
//! @return a hash value computed for the given unsigned integer, in range [1, theUpperBound]
inline int HashCode (const unsigned int theValue, const int theUpperBound)
{
  return mobius::occ::HashCode (static_cast<int> (theValue), theUpperBound);
}

//! Computes a hash code for the given value of the "long long int" type, in range [1, theUpperBound]
//! @param theValue the value of the "long long int" type which hash code is to be computed
//! @param theUpperBound the upper bound of the range a computing hash code must be within
//! @return a computed hash code, in range [1, theUpperBound]
inline int HashCode (const long long int theValue, const int theUpperBound)
{
  return IntegerHashCode(theValue, 0x7fffffffffffffff, theUpperBound);
}

#if (defined(_LP64) || defined(__LP64__) || defined(_WIN64)) || defined(__APPLE__)

//! Computes a hash code for the given value of the Standard_Utf32Char type, in the range [1, theUpperBound]
//! @tparam TheUtf32Char the type of the given value (it is Standard_Utf32Char,
//! and must not be the same as "unsigned int", because the overload of the HashCode function
//! for "unsigned int" type is already presented in int.hxx)
//! @param theValue the value of the Standard_Utf32Char type which hash code is to be computed
//! @param theUpperBound the upper bound of the range a computing hash code must be within
//! @return a computed hash code, in the range [1, theUpperBound]
template <typename TheUtf32Char>
typename opencascade::std::enable_if<!opencascade::std::is_same<Standard_Utf32Char, unsigned int>::value
                                       && opencascade::std::is_same<TheUtf32Char, Standard_Utf32Char>::value,
                                     int>::type
HashCode (const TheUtf32Char theValue, const int theUpperBound)
{
  return IntegerHashCode (theValue, IntegerLast(), theUpperBound);
}

// ------------------------------------------------------------------
// IsEqual : Returns Standard_True if two integers are equal
// ------------------------------------------------------------------
inline Standard_Boolean IsEqual (const Standard_Utf32Char theOne,
                                 const Standard_Utf32Char theTwo)
{
  return theOne == theTwo;
}

}
}

#endif

#endif
