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

#include <mobius/Standard_Real.hxx>

#include <math.h>
#include <mobius/Standard_RangeError.hxx>
#include <mobius/Standard_Real.hxx>
#include <mobius/Standard_Stream.hxx>
#include <mobius/Standard_NullValue.hxx>
#include <mobius/Standard_NumericError.hxx>
#include <mobius/occMathDefs.hxx>

using namespace mobius::occ;

static const double ACosLimit = 1. + Epsilon(1.);

//============================================================================
// function : HashCode
// purpose  :
//============================================================================
int mobius::occ::HashCode (const double theReal, const int theUpperBound)
{
  if (theUpperBound < 1)
  {
    throw std::runtime_error("Try to apply HashCode method with negative or null argument.");
  }
  union
  {
    double    R;
    int I[2];
  } U;

  //  U.R = Abs(me); // Treat me = -0.0 ADN 27/11/97
  U.R = theReal;

  return HashCode (U.I[0] ^ U.I[1], theUpperBound);
}

//-------------------------------------------------------------------
// ACos : Returns the value of the arc cosine of a real
//-------------------------------------------------------------------
double mobius::occ::ACos (const double Value) 
{ 
  if ((Value < -ACosLimit) || (Value > ACosLimit)){
    throw std::runtime_error("range error");
  }
  else if (Value > 1.)
  {
    return 0.; //acos(1.)
  }
  else if (Value < -1.)
  {
    return M_PI; //acos(-1.)
  }
  return acos(Value);
}

//-------------------------------------------------------------------
// ACosApprox : Returns the approximate value of the arc cosine of a real.
//              The max error is about 1 degree near Value=0.
//-------------------------------------------------------------------

inline double apx_for_ACosApprox (const double x)
{
  return  (-0.000007239283986332 +
    x * (2.000291665285952400 +
    x * (0.163910606547823220 +
    x * (0.047654245891495528 -
    x * (0.005516443930088506 +
    0.015098965761299077 * x))))) / sqrt(2*x);
}

double mobius::occ::ACosApprox (const double Value)
{
  double XX;
  if (Value < 0.) {
    XX = 1.+Value;
    if (XX < RealSmall())
      return 0.;
    return M_PI - apx_for_ACosApprox(XX);
  }
  XX = 1.-Value;
  if (XX < RealSmall())
    return 0.;
  return apx_for_ACosApprox(XX);

// The code above is the same but includes 2 comparisons instead of 3
//   double xn = 1.+Value;
//   double xp = 1.-Value;
//   if (xp < RealSmall() || xn < RealSmall())
//     return 0.;
//   if (Value < 0.)
//     return M_PI - apx_for_ACosApprox (xn);
//   return apx_for_ACosApprox (xp);
}

//-------------------------------------------------------------------
// ASin : Returns the value of the arc sine of a real
//-------------------------------------------------------------------
double mobius::occ::ASin (const double Value) 
{ 
  if ((Value < -ACosLimit) || (Value > ACosLimit)){
    throw Standard_RangeError();
  }
  else if (Value > 1.)
  {
    return M_PI_2; //asin(1.)
  }
  else if (Value < -1.)
  {
    return -M_PI_2; //asin(-1.)
  }
  return asin(Value);
}

//-------------------------------------------------------------------
// ATan2 : Returns the arc tangent of a real divide by an another real
//-------------------------------------------------------------------
double mobius::occ::ATan2 (const double Value, const double Other) 
{ 
  if ( Value == 0. && Other == 0. ){
    throw Standard_NullValue();
  }
  return atan2(Value,Other); 
}

//-------------------------------------------------------------------
// Sign : Returns |a| if B >= 0; -|a| if b < 0.
//-------------------------------------------------------------------
double mobius::occ::Sign(const double a, const double b)
{
  if (b >= 0.0) {
    return Abs(a);
  } else {
    return (-1.0 * Abs(a));
  }
}

//==========================================================================
//===== The special routines for "IEEE" and different hardware =============
//==========================================================================
union RealMap {
  double real;
  unsigned int map[2];
};

//--------------------------------------------------------------------
// HardwareHighBitsOfDouble :  
//    Returns 1 if the low bits are at end.   (example: decmips and ALPHA )
//    Returns 0 if the low bits are at begin. (example: sun, sgi, ...)
//--------------------------------------------------------------------
static int HardwareHighBitsOfDouble()
{
  RealMap MaxDouble;
  MaxDouble.real = DBL_MAX;
  //=========================================================
  // representation of the max double in IEEE is
  //      "7fef ffff ffff ffff"   for the big endians.
  //      "ffff ffff 7fef ffff"   for the little endians.
  //=========================================================

  if(MaxDouble.map[1] != 0xffffffff){
    return 1;
  } else {
    return 0;
  }
}

//--------------------------------------------------------------------
// HardwareLowBitsOfDouble :  
//    Returns 0 if the low bits are at end.   (example: decmips )
//    Returns 1 if the low bits are at begin. (example: sun, sgi, ...)
//--------------------------------------------------------------------
static int HardwareLowBitsOfDouble()
{
  RealMap MaxDouble;
  MaxDouble.real = DBL_MAX;
  //=========================================================
  // representation of the max double in IEEE is
  //      "7fef ffff ffff ffff"   for the big endians.
  //      "ffff ffff 7fef ffff"   for the little endians.
  //=========================================================

  if(MaxDouble.map[1] != 0xffffffff){
    return 0;
  } else {
    return 1;
  }
}

static int HighBitsOfDouble = HardwareHighBitsOfDouble();
static int LowBitsOfDouble = HardwareLowBitsOfDouble();

double mobius::occ::NextAfter(const double x, const double y)
{
  RealMap res;

  res.real=x;
  
  if (x == 0.0) {
	return DBL_MIN;
  }
  if(x==y) {
    //=========================================
    //   -oo__________0___________+oo
    //               x=y
    //  The direction is "Null", so there is nothing after
    //=========================================

  } else if (((x<y) && (x>=0.0)) || ((x>y) && (x<0.0))) {
    //=========================================
    //   -oo__________0___________+oo
    //        y <- x     x -> y
    //
    //=========================================
    if (res.map[LowBitsOfDouble]==0xffffffff) {
      res.map[LowBitsOfDouble]=0;
      res.map[HighBitsOfDouble]++;
    } else {
      res.map[LowBitsOfDouble]++;
    }
  } else {
    //=========================================
    //   -oo__________0___________+oo
    //        x -> y     y <- x
    //
    //=========================================
    if (res.map[LowBitsOfDouble]==0) {
      if (res.map[HighBitsOfDouble]==0) {
	res.map[HighBitsOfDouble]=0x80000000;
	res.map[LowBitsOfDouble]=0x00000001;
      } else {
	res.map[LowBitsOfDouble]=0xffffffff;
	res.map[HighBitsOfDouble]--;
      }
    } else {
      res.map[LowBitsOfDouble]--;
    }
  }
  return res.real;
}

//-------------------------------------------------------------------
// ATanh : Returns the value of the hyperbolic arc tangent of a real
//-------------------------------------------------------------------
double     mobius::occ::ATanh(const double Value) 
{ 
  if ( (Value <= -1.) || (Value >= 1.) ){
#ifdef OCCT_DEBUG
    std::cout << "Illegal argument in ATanh" << std::endl ;
#endif
    throw Standard_NumericError("Illegal argument in ATanh");
  }
#if defined(__QNX__)
  return std::atanh(Value);
#else
  return atanh(Value);
#endif
}

//-------------------------------------------------------------------
// ACosh : Returns the hyperbolic Arc cosine of a real
//-------------------------------------------------------------------
double     mobius::occ::ACosh (const double Value) 
{ 
  if ( Value < 1. ){
#ifdef OCCT_DEBUG
    std::cout << "Illegal argument in ACosh" << std::endl ;
#endif
    throw Standard_NumericError("Illegal argument in ACosh");
  }
#if defined(__QNX__)
  return std::acosh(Value);
#else
  return acosh(Value);
#endif
}

//-------------------------------------------------------------------
// Cosh : Returns the hyperbolic cosine of a real
//-------------------------------------------------------------------
double     mobius::occ::Cosh (const double Value) 
{ 
  if ( Abs(Value) > 0.71047586007394394e+03 ){
#ifdef OCCT_DEBUG
    std::cout << "Result of Cosh exceeds the maximum value double" << std::endl ;
#endif
    throw Standard_NumericError("Result of Cosh exceeds the maximum value double");
  } 
  return cosh(Value); 
}

//-------------------------------------------------------------------
// Sinh : Returns the hyperbolicsine of a real
//-------------------------------------------------------------------
double     mobius::occ::Sinh (const double Value) 
{ 
  if ( Abs(Value) > 0.71047586007394394e+03 ){
#ifdef OCCT_DEBUG
    std::cout << "Result of Sinh exceeds the maximum value double" << std::endl ;
#endif
    throw Standard_NumericError("Result of Sinh exceeds the maximum value double");
  } 
  return sinh(Value); 
}

//-------------------------------------------------------------------
// Log : Returns the naturaOPl logarithm of a real
//-------------------------------------------------------------------
double     mobius::occ::Log (const double Value) 
{   if ( Value <= 0. ){
#ifdef OCCT_DEBUG
    std::cout << "Illegal argument in Log" << std::endl ;
#endif
    throw Standard_NumericError("Illegal argument in Log");
  } 
 return log(Value); 
}
//-------------------------------------------------------------------
// Sqrt : Returns the square root of a real
//-------------------------------------------------------------------
double     mobius::occ::Sqrt (const double Value) 
{ 
  if (  Value < 0. ){
#ifdef OCCT_DEBUG
    std::cout << "Illegal argument in Sqrt" << std::endl ;
#endif
    throw Standard_NumericError("Illegal argument in Sqrt");
  } 
 return sqrt(Value); 
}

