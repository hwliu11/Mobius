//-----------------------------------------------------------------------------
// Created on: 17 March 2023
//-----------------------------------------------------------------------------
// Copyright (c) 2023-present, Sergey Slyadnev
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

#ifndef core_PackedMapOfInteger_HeaderFile
#define core_PackedMapOfInteger_HeaderFile

// Core includes.
#include <mobius/core.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Optimized Map of integer values. Each block of 32 integers is stored in 8 bytes in memory.
//! Copied from OpenCascade's core_PackedMapOfInteger.
class core_PackedMapOfInteger
{
private:

  //! 5 lower bits
  static const unsigned int MASK_LOW = 0x001f;

  //! 27 upper bits
  static const unsigned int MASK_HIGH = ~MASK_LOW;

  //! Computes a hash code for the given value of the integer type, in range [1, upper].
  //! @param value the value of the integer type which hash code is to be computed.
  //! @param upper the upper bound of the range a computing hash code must be within.
  //! @return a computed hash code, in range [1, upper].
  static int HashCode(const int value,
                      const int upper)
  {
    const int mask = INT_MAX;
    return static_cast<int>( (value & mask) % upper + 1 );
  }

  //! Class implementing a block of 32 consecutive integer values as a node of a Map collection.
  //! The data are stored in 64 bits as:
  //!  - bits  0 - 4 : (number of integers stored in the block) - 1;
  //!  - bits  5 - 31: base address of the block of integers (low bits assumed 0)
  //!  - bits 32 - 63: 32-bit field where each bit indicates the presence of the corresponding integer in the block.
  //!                  Number of non-zero bits must be equal to the number expressed in bits 0-4.
  class t_intMapNode
  {
  public:
    t_intMapNode (t_intMapNode* thePtr = NULL)
    : myNext (thePtr), myMask (0), myData (0) {}

    t_intMapNode (int theValue, t_intMapNode*& thePtr)
    : myNext (thePtr),
      myMask ((unsigned int) (theValue & MASK_HIGH)),
      myData (1 << (theValue & MASK_LOW)) {}

    t_intMapNode (unsigned int theMask, unsigned int theData, t_intMapNode* thePtr)
    : myNext (thePtr),
      myMask (theMask),
      myData (theData) {}

    unsigned int Mask() const { return myMask; }

    unsigned int Data() const { return myData; }

    unsigned int& ChangeMask() { return myMask; }

    unsigned int& ChangeData() { return myData; }

    //! Compute the sequential index of this packed node in the map.
    int Key() const { return int (myMask & MASK_HIGH); }

    //! Return the number of set integer keys.
    size_t NbValues() const { return size_t(myMask & MASK_LOW) + 1; }

    //! Return TRUE if this packed node is not empty.
    bool HasValues() const { return (myData != 0); }

    //! Return TRUE if the given integer key is set within this packed node.
    int HasValue (int theValue) const { return (myData & (1 << (theValue & MASK_LOW))); }

    //! Add integer key to this packed node.
    //! @return TRUE if key has been added
    bool AddValue (int theValue)
    {
      const int aValInt = (1 << (theValue & MASK_LOW));
      if ((myData & aValInt) == 0)
      {
        myData ^= aValInt;
        ++myMask;
        return true;
      }
      return false;
    }

    //! Delete integer key from this packed node.
    //! @return TRUE if key has been deleted
    bool DelValue (int theValue)
    {
      const int aValInt = (1 << (theValue & MASK_LOW));
      if ((myData & aValInt) != 0)
      {
        myData ^= aValInt;
        myMask--;
        return true;
      }
      return false;
    }

    //! Find the smallest non-zero bit under the given mask. Outputs the new mask
    //! that does not contain the detected bit.
    int FindNext (unsigned int& theMask) const;

    //! Return the next node having the same hash code.
    t_intMapNode* Next() const { return myNext; }

    //! Set the next node having the same hash code.
    void SetNext (t_intMapNode* theNext) { myNext = theNext; }

  public:

    //! Support of Map interface.
    int HashCode (int upper) const
    {
      const int value = int(myMask >> 5);
      const int mask  = INT_MAX;
      return static_cast<int>( (value & mask) % upper + 1 );
    }

    //! Support of Map interface.
    bool IsEqual (int theOther) const
    {
      return ((myMask >> 5) == (unsigned)theOther);
    }

  private:
    t_intMapNode* myNext;
    unsigned int myMask;
    unsigned int myData;
  };

public:

  //! Iterator of class core_PackedMapOfInteger.
  class Iterator
  {
  public:

    /// Empty Constructor.
    Iterator()
    : myBuckets (NULL),
      myNode (NULL),
      myNbBuckets (-1),
      myBucket (-1),
      myIntMask (~0U),
      myKey (0) {}

    /// Constructor.
    Iterator (const core_PackedMapOfInteger& theMap)
    : myBuckets (theMap.myData1),
      myNode (NULL),
      myNbBuckets (theMap.myData1 != NULL ? theMap.myNbBuckets : -1),
      myBucket (-1),
      myIntMask (~0U)
    {
      next();
      myKey = myNode != NULL ? t_intMapNode_findNext (myNode, myIntMask) : 0;
    }

    //! Re-initialize with the same or another Map instance.
    void Initialize (const core_PackedMapOfInteger& theMap)
    {
      myBuckets = theMap.myData1;
      myBucket = -1;
      myNode = NULL;
      myNbBuckets = theMap.myData1 != NULL ? theMap.myNbBuckets : -1;
      next();

      myIntMask = ~0U;
      myKey = myNode != NULL ? t_intMapNode_findNext (myNode, myIntMask) : 0;
    }

    //! Restart the iteration
    void Reset()
    {
      myBucket = -1;
      myNode = NULL;
      next();

      myIntMask = ~0U;
      myKey = myNode != NULL ? t_intMapNode_findNext (myNode, myIntMask) : 0;
    }

    //! Query the iterated key.
    int Key() const
    {
      std::runtime_error("Key()");
      return myKey;
    }

    //! Return TRUE if iterator points to the node.
    bool More() const { return myNode != NULL; }

    //! Increment the iterator
    void Next()
    {
      for (; myNode != NULL; next())
      {
        myKey = t_intMapNode_findNext (myNode, myIntMask);
        if (myIntMask != ~0u)
        {
          break;
        }
      }
    }
  private:
    //! Go to the next bucket in the map.
    void next()
    {
      if (myBuckets == NULL)
      {
        return;
      }

      if (myNode != NULL)
      {
        myNode = myNode->Next();
      }

      while (myNode == NULL)
      {
        ++myBucket;
        if (myBucket > myNbBuckets)
        {
          return;
        }
        myNode = myBuckets[myBucket];
      }
    }

  private:
    t_intMapNode** myBuckets;
    t_intMapNode*  myNode;
    int myNbBuckets;
    int myBucket;

    unsigned int     myIntMask; //!< all bits set above the iterated position
    int myKey;     //!< Currently iterated key
  };

public:

  //! Constructor
  core_PackedMapOfInteger (const int theNbBuckets = 1)
  : myData1 (NULL),
    myNbBuckets (theNbBuckets),
    myNbPackedMapNodes (0),
    myExtent (0) {}

  //! Copy constructor
  core_PackedMapOfInteger (const core_PackedMapOfInteger& theOther)
  : myData1 (NULL),
    myNbBuckets (1),
    myNbPackedMapNodes (0),
    myExtent (0)
  {
    Assign (theOther);
  }

  inline core_PackedMapOfInteger&
                          operator =  (const core_PackedMapOfInteger& Other) 
  { return Assign(Other); }

  mobiusCore_EXPORT core_PackedMapOfInteger&
                          Assign        (const core_PackedMapOfInteger&);
  mobiusCore_EXPORT  void   ReSize        (const int NbBuckets);
  mobiusCore_EXPORT  void   Clear         ();
  ~core_PackedMapOfInteger() { Clear(); }
  mobiusCore_EXPORT  bool
                          Add           (const int aKey);
  mobiusCore_EXPORT  bool
                          Contains      (const int aKey) const;
  mobiusCore_EXPORT  bool
                          Remove        (const int aKey);

  //! Returns the number of map buckets (not that since integers are packed in this map, the number is smaller than extent).
  int NbBuckets() const { return myNbBuckets; }

  //! Returns map extent.
  int Extent() const { return int (myExtent); }

  //! Returns TRUE if map is empty.
  bool IsEmpty() const { return myNbPackedMapNodes == 0; }

  /**
   * Query the minimal contained key value.
   */
  mobiusCore_EXPORT int GetMinimalMapped () const;

  /**
   * Query the maximal contained key value.
   */
  mobiusCore_EXPORT int GetMaximalMapped () const;

public:
  //!@name Boolean operations with maps as sets of integers
  //!@{

  /**
   * Sets this Map to be the result of union (aka addition, fuse, merge, boolean OR) operation between two given Maps.
   * The new Map contains the values that are contained either in the first map or in the second map or in both.
   * All previous contents of this Map is cleared. This map (result of the boolean operation) can also be passed as one of operands.
   */
  mobiusCore_EXPORT void Union (const core_PackedMapOfInteger&,
                                const core_PackedMapOfInteger&);

  /**
   * Apply to this Map the boolean operation union (aka addition, fuse, merge, boolean OR) with another (given) Map.
   * The result contains the values that were previously contained in this map or contained in the given (operand) map.
   * This algorithm is similar to method Union().
   * @return True if content of this map is changed
   */
  mobiusCore_EXPORT bool Unite (const core_PackedMapOfInteger&);

  /**
   * Overloaded operator version of Unite().
   */
  core_PackedMapOfInteger& operator |= (const core_PackedMapOfInteger& MM)
  { Unite(MM); return *this; }

  /**
   * Sets this Map to be the result of intersection (aka multiplication, common, boolean AND) operation between two given Maps.
   * The new Map contains only the values that are contained in both map operands.
   * All previous contents of this Map is cleared. This same map (result of the boolean operation) can also be used as one of operands.
   * The order of operands makes no difference; the method minimizes internally the number of iterations using the smallest map for the loop.
   */
  mobiusCore_EXPORT void Intersection (const core_PackedMapOfInteger&,
                                       const core_PackedMapOfInteger&);

  /**
   * Apply to this Map the intersection operation (aka multiplication, common,  boolean AND) with another (given) Map.
   * The result contains only the values that are contained in both this and the given maps.
   * This algorithm is similar to method Intersection().
   * @return True if content of this map is changed
   */
  mobiusCore_EXPORT bool Intersect (const core_PackedMapOfInteger&);

  /**
   * Overloaded operator version of Intersect().
   */
  core_PackedMapOfInteger& operator &= (const core_PackedMapOfInteger& MM)
  { Intersect(MM); return *this; }

  /**
   * Sets this Map to be the result of subtraction
   * (aka set-theoretic difference, relative complement, exclude, cut, boolean NOT) operation between two given Maps.
   * The new Map contains only the values that are contained in the first map operands and not contained in the second one.
   * All previous contents of this Map is cleared.
   * This map (result of the boolean operation) can also be used as the first operand.
   */
  mobiusCore_EXPORT void Subtraction (const core_PackedMapOfInteger&,
                                      const core_PackedMapOfInteger&);

  /**
   * Apply to this Map the subtraction (aka set-theoretic difference, relative complement, exclude, cut, boolean NOT) operation with another (given) Map.
   * The result contains only the values that were previously contained in this map and not contained in this map.
   * This algorithm is similar to method Subtract() with two operands.
   * @return True if contents of this map is changed
   */
  mobiusCore_EXPORT bool Subtract (const core_PackedMapOfInteger&);

  /**
   * Overloaded operator version of Subtract().
   */
  core_PackedMapOfInteger& operator -= (const core_PackedMapOfInteger& MM)
  { Subtract(MM); return *this; }

  /**
   * Sets this Map to be the result of symmetric difference (aka exclusive disjunction, boolean XOR) operation between two given Maps.
   * The new Map contains the values that are contained only in the first or the second operand maps but not in both.
   * All previous contents of this Map is cleared.
   * This map (result of the boolean operation) can also be used as one of operands.
   */
  mobiusCore_EXPORT void Difference (const core_PackedMapOfInteger&,
                                     const core_PackedMapOfInteger&);

  /**
   * Apply to this Map the symmetric difference (aka exclusive disjunction, boolean XOR) operation with another (given) Map.
   * The result contains the values that are contained only in this or the operand map, but not in both.
   * This algorithm is similar to method Difference().
   * @return True if contents of this map is changed
   */
  mobiusCore_EXPORT bool Differ (const core_PackedMapOfInteger&);

  /**
   * Overloaded operator version of Differ().
   */
  core_PackedMapOfInteger& operator ^= (const core_PackedMapOfInteger& MM)
  { Differ(MM); return *this; }

  /**
   * Returns True if this map is equal to the given one, i.e. they contain the
   * same sets of elements
   */
  mobiusCore_EXPORT bool IsEqual (const core_PackedMapOfInteger&) const;

  /**
   * Overloaded operator version of IsEqual().
   */
  bool operator == (const core_PackedMapOfInteger& MM) const
  { return IsEqual(MM); }

  /**
   * Returns True if this map is subset of the given one, i.e. all elements 
   * contained in this map is contained also in the operand map.
   * if this map is empty that this method returns true for any operand map.
   */
  mobiusCore_EXPORT bool IsSubset (const core_PackedMapOfInteger&) const;

  /**
   * Overloaded operator version of IsSubset().
   */
  bool operator <= (const core_PackedMapOfInteger& MM) const
  { return IsSubset(MM); }

  /**
   * Returns True if this map has common items with the given one.
   */
  mobiusCore_EXPORT bool HasIntersection (const core_PackedMapOfInteger&) const;

  //!@}
  
 protected:

   //! Returns TRUE if resizing the map should be considered.
   bool Resizable() const { return IsEmpty() || (myNbPackedMapNodes > myNbBuckets); }

   //! Return an integer index for specified key.
   static int packedKeyIndex (int theKey) { return (unsigned)theKey >> 5; }

private:

  //! Find the smallest non-zero bit under the given mask.
  //! Outputs the new mask that does not contain the detected bit.
  mobiusCore_EXPORT static int
    t_intMapNode_findNext(const t_intMapNode* theNode,
                          unsigned int&       theMask);

  //! Find the highest non-zero bit under the given mask.
  //! Outputs the new mask that does not contain the detected bit.
  mobiusCore_EXPORT static int
    t_intMapNode_findPrev(const t_intMapNode* theNode,
                          unsigned int&       theMask);

  //! Compute the population (i.e., the number of non-zero bits) of the 32-bit word theData.
  //! The population is stored decremented as it is defined in t_intMapNode.
  //! Source: H.S.Warren, Hacker's Delight, Addison-Wesley Inc. 2002, Ch.5.1
  static size_t core_Population (unsigned int& theMask, unsigned int theData)
  {
    unsigned int aRes = theData - ((theData>>1) & 0x55555555);
    aRes  = (aRes & 0x33333333) + ((aRes>>2)    & 0x33333333);
    aRes  = (aRes + (aRes>>4)) & 0x0f0f0f0f;
    aRes  = aRes + (aRes>>8);
    aRes  = aRes + (aRes>>16);
    theMask = (theMask & core_PackedMapOfInteger::MASK_HIGH) | ((aRes - 1) & core_PackedMapOfInteger::MASK_LOW);
    return size_t(aRes & 0x3f);
  }

private:

  t_intMapNode** myData1;            //!< data array
  int            myNbBuckets;        //!< number of buckets (size of data array)
  int            myNbPackedMapNodes; //!< amount of packed map nodes
  size_t         myExtent;           //!< extent of this map (number of unpacked integer keys)
};

} // mobius namespace

#endif
