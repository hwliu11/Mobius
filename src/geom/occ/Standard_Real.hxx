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

#ifndef _Standard_Real_HeaderFile
#define _Standard_Real_HeaderFile

#include <cmath>
#include <float.h>
#include <mobius/Standard_Values.hxx>
#include <mobius/Standard_math.hxx>
#include <mobius/Standard_TypeDef.hxx>

namespace mobius {
namespace occ {

// ===============================================
// Methods from Standard_Entity class which are redefined:  
//    - Hascode
//    - IsEqual
// ===============================================

// ==================================
// Methods implemented in double.cxx
// ==================================

//! Computes a hash code for the given real, in the range [1, theUpperBound]
//! @param theReal the real value which hash code is to be computed
//! @param theUpperBound the upper bound of the range a computing hash code must be within
//! @return a computed hash code, in the range [1, theUpperBound]
mobiusGeom_EXPORT int HashCode    (double theReal, int theUpperBound);

mobiusGeom_EXPORT double    ACos        (const double );
mobiusGeom_EXPORT double    ACosApprox  (const double );
mobiusGeom_EXPORT double    ASin        (const double );
mobiusGeom_EXPORT double    ATan2       (const double , const double );
mobiusGeom_EXPORT double    NextAfter   (const double , const double );

//! Returns |a| if b >= 0; -|a| if b < 0.
mobiusGeom_EXPORT double    Sign(const double a, const double b);

mobiusGeom_EXPORT double    ATanh       (const double );
mobiusGeom_EXPORT double    ACosh       (const double );
mobiusGeom_EXPORT double    Sinh       (const double );
mobiusGeom_EXPORT double    Cosh       (const double );
mobiusGeom_EXPORT double    Log         (const double );
mobiusGeom_EXPORT double    Sqrt        (const double );

//-------------------------------------------------------------------
// RealSmall : Returns the smallest positive real
//-------------------------------------------------------------------
inline double     RealSmall() 
{ return DBL_MIN; }

//-------------------------------------------------------------------
// Abs : Returns the absolute value of a real
//-------------------------------------------------------------------
inline double     Abs(const double Value) 
{ return fabs(Value); }


//-------------------------------------------------------------------
// IsEqual : Returns Standard_True if two reals are equal
//-------------------------------------------------------------------
inline Standard_Boolean  IsEqual (const double Value1, 
				  const double Value2) 
{ return Abs((Value1 - Value2)) < RealSmall(); }

         //  *********************************** //
         //       Class methods                  //
         //                                      //
         //  Machine-dependent values            //
         //  Should be taken from include file   //
         //  *********************************** //


//-------------------------------------------------------------------
// RealDigit : Returns the number of digits of precision in a real
//-------------------------------------------------------------------
inline int  RealDigits() 
{ return DBL_DIG; }

//-------------------------------------------------------------------
// RealEpsilon : Returns the minimum positive real such that 
//               1.0 + x is not equal to 1.0
//-------------------------------------------------------------------
inline double     RealEpsilon() 
{ return DBL_EPSILON; }

//-------------------------------------------------------------------
// RealFirst : Returns the minimum negative value of a real
//-------------------------------------------------------------------
inline double     RealFirst() 
{ return -DBL_MAX; }
  
//-------------------------------------------------------------------
// RealFirst10Exp : Returns the minimum value of exponent(base 10) of
//                  a real.
//-------------------------------------------------------------------
inline int  RealFirst10Exp() 
{ return DBL_MIN_10_EXP; }

//-------------------------------------------------------------------
// RealLast : Returns the maximum value of a real
//-------------------------------------------------------------------
inline double     RealLast() 
{ return  DBL_MAX; }

//-------------------------------------------------------------------
// RealLast10Exp : Returns the maximum value of exponent(base 10) of
//                 a real.
//-------------------------------------------------------------------
inline int  RealLast10Exp() 
{ return  DBL_MAX_10_EXP; }

//-------------------------------------------------------------------
// RealMantissa : Returns the size in bits of the matissa part of a 
//                real.
//-------------------------------------------------------------------
inline int  RealMantissa() 
{ return  DBL_MANT_DIG; }

//-------------------------------------------------------------------
// RealRadix : Returns the radix of exponent representation
//-------------------------------------------------------------------
inline int  RealRadix() 
{ return  FLT_RADIX; }

//-------------------------------------------------------------------
// RealSize : Returns the size in bits of an integer
//-------------------------------------------------------------------
inline int  RealSize() 
{ return BITS(double); }



         //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=//
         //   End of machine-dependent values   //
         //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=//


//-------------------------------------------------------------------
// IntToReal : Converts an integer in a real
//-------------------------------------------------------------------
inline double     IntToReal(const int Value) 
{ return Value; }

//-------------------------------------------------------------------
// ATan : Returns the value of the arc tangent of a real
//-------------------------------------------------------------------
inline double     ATan(const double Value) 
{ return atan(Value); }


//-------------------------------------------------------------------
// Ceiling : Returns the smallest integer not less than a real
//-------------------------------------------------------------------
inline double     Ceiling (const double Value) 
{ return ceil(Value); }

//-------------------------------------------------------------------
// Cos : Returns the cosine of a real
//-------------------------------------------------------------------
inline double     Cos (const double Value) 
{ return cos(Value); }


//-------------------------------------------------------------------
// Epsilon : The function returns absolute value of difference
//           between 'Value' and other nearest value of
//           double type.
//           Nearest value is chosen in direction of infinity
//           the same sign as 'Value'.
//           If 'Value' is 0 then returns minimal positive value
//           of double type.
//-------------------------------------------------------------------
inline double     Epsilon (const double Value) 
{
  double aEpsilon;

  if (Value>=0.0){
    aEpsilon = NextAfter(Value, RealLast()) - Value;
  } else {
    aEpsilon = Value - NextAfter(Value, RealFirst());
  }
  return aEpsilon;
}

//-------------------------------------------------------------------
// Exp : Returns the exponential function of a real
//-------------------------------------------------------------------
inline double     Exp (const double Value) 
{ return exp(Value); }

//-------------------------------------------------------------------
// Floor : Return the largest integer not greater than a real
//-------------------------------------------------------------------
inline double     Floor (const double Value) 
{ return floor(Value); }

//-------------------------------------------------------------------
// IntegerPart : Returns the integer part of a real
//-------------------------------------------------------------------
inline double     IntegerPart (const double Value) 
{ return ( (Value>0) ? floor(Value) : ceil(Value) ); }


//-------------------------------------------------------------------
// Log10 : Returns the base-10 logarithm of a real 
//-------------------------------------------------------------------
inline double     Log10 (const double Value) 
{ return log10(Value); }

//-------------------------------------------------------------------
// Max : Returns the maximum value of two reals
//-------------------------------------------------------------------
inline double     Max (const double Val1, 
                              const double Val2) 
{
  return Val1 >= Val2 ? Val1 : Val2;
}

//-------------------------------------------------------------------
// Min : Returns the minimum value of two reals
//-------------------------------------------------------------------
inline double     Min (const double Val1, 
                              const double Val2)
{
  return Val1 <= Val2 ? Val1 : Val2;
}

//-------------------------------------------------------------------
// Pow : Returns a real to a given power
//-------------------------------------------------------------------
inline double     Pow (const double Value, const double P)
{ return pow(Value,P); }

//-------------------------------------------------------------------
// RealPart : Returns the fractional part of a real.
//-------------------------------------------------------------------
inline  double    RealPart (const double Value) 
{ return fabs(IntegerPart(Value) - Value); }

//-------------------------------------------------------------------
// RealToInt : Returns the real converted to nearest valid integer.
//             If input value is out of valid range for integers,
//             minimal or maximal possible integer is returned.
//-------------------------------------------------------------------
inline  int RealToInt (const double Value) 
{ 
  // Note that on WNT under MS VC++ 8.0 conversion of double value less 
  // than INT_MIN or greater than INT_MAX to integer will cause signal 
  // "Floating point multiple trap" (OCC17861)
  return Value < INT_MIN ? INT_MIN
    : Value > INT_MAX ? INT_MAX
    : (int)Value;
}

// =======================================================================
// function : RealToShortReal
// purpose  : Converts double value to the nearest valid
//            Standard_ShortReal. If input value is out of valid range
//            for Standard_ShortReal, minimal or maximal
//            Standard_ShortReal is returned.
// =======================================================================
inline Standard_ShortReal RealToShortReal (const double theVal)
{
  return theVal < -FLT_MAX ? -FLT_MAX
    : theVal > FLT_MAX ? FLT_MAX
    : (Standard_ShortReal)theVal;
}

//-------------------------------------------------------------------
// Round : Returns the nearest integer of a real
//-------------------------------------------------------------------
inline double     Round (const double Value) 
{ return IntegerPart(Value + (Value > 0 ? 0.5 : -0.5)); }

//-------------------------------------------------------------------
// Sin : Returns the sine of a real
//-------------------------------------------------------------------
inline double     Sin (const double Value) 
{ return sin(Value); }


//-------------------------------------------------------------------
// ASinh : Returns the hyperbolic arc sine of a real
//-------------------------------------------------------------------
inline double     ASinh(const double Value)
#if defined(__QNX__)
{ return std::asinh(Value); }
#else
{ return asinh(Value); }
#endif

//-------------------------------------------------------------------
// Square : Returns a real to the power 2
//-------------------------------------------------------------------
inline double     Square(const double Value) 
{ return Value * Value; }

//-------------------------------------------------------------------
// Tan : Returns the tangent of a real
//-------------------------------------------------------------------
inline double     Tan (const double Value) 
{ return tan(Value); }

//-------------------------------------------------------------------
// Tanh : Returns the hyperbolic tangent of a real
//-------------------------------------------------------------------
inline double     Tanh (const double Value) 
{ return tanh(Value); }

}
}

#endif
