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

// Own include
#include <mobius/core_PackedMapOfInteger.h>

// Standard includes.
#include <cmath>
#include <float.h>

using namespace mobius;

//-----------------------------------------------------------------------------

// The array of prime numbers used as consecutive steps for
// size of array of buckets in the map.
// The prime numbers are used for array size with the hope that this will
// lead to less probablility of having the same hash codes for
// different map items (note that all hash codes are modulo that size).
// The value of each next step is chosen to be ~2 times greater than previous.
// Though this could be thought as too much, actually the amount of 
// memory overhead in that case is only ~15% as compared with total size of
// all auxiliary data structures (each map node takes ~24 bytes), 
// and this proves to pay off in performance (see OCC13189).
#define THE_NB_PRIMES 24
static const int THE_TCollection_Primes[THE_NB_PRIMES] =
{
         101,
        1009,
        2003,
        5003,
       10007,
       20011,
       37003,
       57037,
       65003,
      100019,
      209953, // The following are Pierpont primes [List of prime numbers]
      472393,
      995329,
     2359297,
     4478977,
     9437185,
    17915905,
    35831809,
    71663617,
   150994945,
   301989889,
   573308929,
  1019215873,
  2038431745
};

namespace Standard
{
  void* Allocate(const size_t aSize)
  {
    // the size is rounded up to 4 since some OCC classes
    // (e.g. TCollection_AsciiString) assume memory to be double word-aligned
    const size_t aRoundSize = (aSize + 3) & ~0x3;
    // we use ?: operator instead of if() since it is faster :-)
    void* aPtr = malloc(aRoundSize);
    if ( ! aPtr )
      throw std::runtime_error("Standard::Allocate(): malloc failed");
    return aPtr;
  }

  void Free(void* thePtr)
  {
    free(thePtr);
  }
}

namespace TCollection {

  int NextPrimeForMap(const int N)
  {
    for (int aPrimeIter = 0; aPrimeIter < THE_NB_PRIMES; ++aPrimeIter)
    {
      if (THE_TCollection_Primes[aPrimeIter] > N)
      {
        return THE_TCollection_Primes[aPrimeIter];
      }
    }
    throw std::runtime_error("TCollection::NextPrimeForMap() - requested too big size");
  }

}

//=======================================================================
//function : t_intMapNode_findNext
//purpose  :
//=======================================================================
int core_PackedMapOfInteger::t_intMapNode_findNext (const t_intMapNode* theNode,
                                                                          unsigned int& theMask)
{
  const t_intMapNode* aNode = reinterpret_cast<const t_intMapNode*> (theNode);
  unsigned int val = aNode->Data() & theMask;
  int nZeros (0);
  if (val == 0)
    theMask = ~0U;   // void, nothing to do
  else{
    unsigned int aMask = ~0U;
    if ((val & 0x0000ffff) == 0) {
      aMask = 0xffff0000;
      nZeros = 16;
      val >>= 16;
    }
    if ((val & 0x000000ff) == 0) {
      aMask <<= 8;
      nZeros += 8;
      val >>= 8;
    }
    if ((val & 0x0000000f) == 0) {
      aMask <<= 4;
      nZeros += 4;
      val >>= 4;
    }
    if ((val & 0x00000003) == 0) {
      aMask <<= 2;
      nZeros += 2;
      val >>= 2;
    }
    if ((val & 0x00000001) == 0) {
      aMask <<= 1;
      nZeros ++;
    }
    theMask = (aMask << 1);
  }
  return nZeros + aNode->Key();
}

//=======================================================================
//function : t_intMapNode_findPrev
//purpose  :
//=======================================================================
int core_PackedMapOfInteger::t_intMapNode_findPrev (const t_intMapNode* theNode,
                                                                          unsigned int& theMask)
{
  const t_intMapNode* aNode = reinterpret_cast<const t_intMapNode*> (theNode);
  unsigned int val = aNode->Data() & theMask;
  int nZeros (0);
  if (val == 0)
    theMask = ~0U;   // void, nothing to do
  else {
    unsigned int aMask = ~0U;
    if ((val & 0xffff0000) == 0) {
      aMask = 0x0000ffff;
      nZeros = 16;
      val <<= 16;
    }
    if ((val & 0xff000000) == 0) {
      aMask >>= 8;
      nZeros += 8;
      val <<= 8;
    }
    if ((val & 0xf0000000) == 0) {
      aMask >>= 4;
      nZeros += 4;
      val <<= 4;
    }
    if ((val & 0xc0000000) == 0) {
      aMask >>= 2;
      nZeros += 2;
      val <<= 2;
    }
    if ((val & 0x80000000) == 0) {
      aMask >>= 1;
      nZeros ++;
    }
    theMask = (aMask >> 1);
  }
  return (31 - nZeros) + aNode->Key();
}

//=======================================================================
//function : Assign
//purpose  : 
//=======================================================================

core_PackedMapOfInteger& core_PackedMapOfInteger::Assign
                                  (const core_PackedMapOfInteger& theOther)
{
  if (this != &theOther) {
    Clear();
    if  (!theOther.IsEmpty()) { 
      ReSize (theOther.myNbPackedMapNodes);
      const int nBucketsSrc = theOther.myNbBuckets;
      const int nBuckets    = myNbBuckets;
      for (int i = 0; i <= nBucketsSrc; i++)
      {
        for (const t_intMapNode* p = theOther.myData1[i]; p != NULL; )
        {
          const int aHashCode = p->HashCode(nBuckets);
          myData1[aHashCode] = new t_intMapNode (p->Mask(), p->Data(), myData1[aHashCode]);
          ++myNbPackedMapNodes;
          p = p->Next();
        }
      }
//       TColStd_MapIteratorOfPackedMapOfInteger anIt (theOther);
//       for (; anIt.More(); anIt.Next())
//         Add (anIt.Key());
    }
  }
  myExtent  = theOther.myExtent;
  return * this;
}

//=======================================================================
//function : ReSize
//purpose  : 
//=======================================================================

void core_PackedMapOfInteger::ReSize (const int theNbBuckets)
{
  int aNewBuck = TCollection::NextPrimeForMap (theNbBuckets);
  if (aNewBuck <= myNbBuckets)
  {
    if (!IsEmpty())
    {
      return;
    }
    aNewBuck = myNbBuckets;
  }

  t_intMapNode** aNewData = (t_intMapNode** )Standard::Allocate ((aNewBuck + 1) * sizeof(t_intMapNode*));
  memset (aNewData, 0, (aNewBuck + 1) * sizeof(t_intMapNode*));
  if (myData1 != NULL)
  {
    t_intMapNode** anOldData = myData1;
    for (int i = 0; i <= myNbBuckets; ++i)
    {
      for (t_intMapNode* p = anOldData[i]; p != NULL; )
      {
        int k = p->HashCode (aNewBuck);
        t_intMapNode* q = p->Next();
        p->SetNext (aNewData[k]);
        aNewData[k] = p;
        p = q;
      }
    }
  }

  Standard::Free (myData1);
  myNbBuckets = aNewBuck;
  myData1 = aNewData;
}

//=======================================================================
//function : Clear
//purpose  : 
//=======================================================================

void core_PackedMapOfInteger::Clear ()
{
  if (!IsEmpty())
  {
    for (int aBucketIter = 0; aBucketIter <= myNbBuckets; ++aBucketIter)
    {
      if (myData1[aBucketIter])
      {
        for (t_intMapNode* aSubNodeIter = myData1[aBucketIter]; aSubNodeIter != NULL; )
        {
          t_intMapNode* q = aSubNodeIter->Next();
          delete aSubNodeIter;
          aSubNodeIter = q;
        }
      }
    }
  }

  myNbPackedMapNodes = 0;
  Standard::Free (myData1);
  myData1 = NULL;
  myExtent = 0;
}

//=======================================================================
//function : Add
//purpose  : 
//=======================================================================

bool core_PackedMapOfInteger::Add (const int aKey)
{
  if (Resizable())
  {
    ReSize (myNbPackedMapNodes);
  }

  const int aKeyInt = packedKeyIndex (aKey);
  const int aHashCode = HashCode (aKeyInt, myNbBuckets);
  t_intMapNode* aBucketHead = myData1[aHashCode];
  for (t_intMapNode* p = aBucketHead; p != NULL; p = p->Next())
  {
    if (p->IsEqual(aKeyInt))
    {
      if (p->AddValue (aKey))
      {
        ++myExtent;
        return true;
      }
      return false;
    }
  }

  myData1[aHashCode] = new t_intMapNode (aKey, aBucketHead);
  ++myNbPackedMapNodes;
  ++myExtent;
  return true;
}

//=======================================================================
//function : Contains
//purpose  : 
//=======================================================================

bool core_PackedMapOfInteger::Contains
                                        (const int aKey) const
{
  if (IsEmpty())
  {
    return false;
  }

  bool aResult = false;
  const int aKeyInt = packedKeyIndex (aKey);
  for (t_intMapNode* p = myData1[HashCode (aKeyInt, myNbBuckets)]; p != NULL; )
  {
    if (p->IsEqual(aKeyInt))
    {
      aResult = (p->HasValue (aKey) != 0);
      break;
    }
    p = p->Next();
  }
  return aResult;
}

//=======================================================================
//function : Remove
//purpose  : 
//=======================================================================

bool core_PackedMapOfInteger::Remove(const int aKey)
{
  if (IsEmpty())
  {
    return false;
  }

  bool aResult (false);
  const int aKeyInt = packedKeyIndex (aKey);
  t_intMapNode*& aBucketHead = myData1[HashCode(aKeyInt, myNbBuckets)];
  t_intMapNode*  p = aBucketHead;
  t_intMapNode*  q = 0L;
  while (p)
  {
    if (p->IsEqual(aKeyInt))
    {
      aResult = p->DelValue (aKey);
      if (aResult)
      {
        --myExtent;
        if (!p->HasValues())
        {
          --myNbPackedMapNodes;
          if (q != NULL)
          {
            q->SetNext (p->Next());
          }
          else
          {
            aBucketHead = p->Next();
          }
          delete p;
        }
      }
      break;
    }
    q = p;
    p = p->Next();
  }
  return aResult;
}

//=======================================================================
//function : GetMinimalMapped
//purpose  : Query the minimal contained key value.
//=======================================================================

int core_PackedMapOfInteger::GetMinimalMapped () const
{
  if (IsEmpty())
  {
    return INT_MAX;
  }

  int aResult = INT_MAX;
  const t_intMapNode* pFoundNode = 0L;
  for (int i = 0; i <= myNbBuckets; i++)
  {
    for (const t_intMapNode* p = myData1[i]; p != 0L; p = p->Next())
    {
      const int aKey = p->Key();
      if (aResult > aKey)
      {
        aResult = aKey;
        pFoundNode = p;
      }
    }
  }
  if (pFoundNode)
  {
    unsigned int aFullMask (0xffffffff);
    aResult = t_intMapNode_findNext (pFoundNode, aFullMask);
  }
  return aResult;
}

//=======================================================================
//function : GetMaximalMapped
//purpose  : Query the maximal contained key value.
//=======================================================================

int core_PackedMapOfInteger::GetMaximalMapped () const
{
  if (IsEmpty())
  {
    return INT_MIN;
  }

  int aResult = INT_MIN;
  const t_intMapNode* pFoundNode = 0L;
  for (int i = 0; i <= myNbBuckets; i++)
  {
    for (const t_intMapNode* p = myData1[i]; p != 0L; p = p->Next())
    {
      const int aKey = p->Key();
      if (aResult < aKey)
      {
        aResult = aKey;
        pFoundNode = p;
      }
    }
  }
  if (pFoundNode)
  {
    unsigned int aFullMask (0xffffffff);
    aResult = t_intMapNode_findPrev (pFoundNode, aFullMask);
  }
  return aResult;
}

//=======================================================================
//function : Union
//purpose  : Boolean operation OR between 2 maps
//=======================================================================

void core_PackedMapOfInteger::Union (const core_PackedMapOfInteger& theMap1,
                                        const core_PackedMapOfInteger& theMap2)
{
  if (theMap1.IsEmpty()) // 0 | B == B
    Assign (theMap2);
  else if (theMap2.IsEmpty()) // A | 0 == A
    Assign (theMap1);
  else if (myData1 == theMap1.myData1)
    Unite (theMap2);
  else if (myData1 == theMap2.myData1)
    Unite (theMap1);
  else {
    const int nBuckets1 = theMap1.myNbBuckets;
    const int nBuckets2 = theMap2.myNbBuckets;
    Clear();
    // Iteration of the 1st map.
    for (int i = 0; i <= nBuckets1; i++) {
      const t_intMapNode* p1 = theMap1.myData1[i];
      while (p1 != 0L) {
        // Find aKey - the base address of currently iterated block
        const int aKey = p1->Key();
        const int aKeyInt = packedKeyIndex (aKey);
        unsigned int aNewMask = p1->Mask();
        unsigned int aNewData = p1->Data();
        size_t       nValues (p1->NbValues());
        // Find the corresponding block in the 2nd map
        const t_intMapNode* p2 = theMap2.myData1[HashCode (aKeyInt, nBuckets2)];
        while (p2) {
          if (p2->IsEqual(aKeyInt)) {
            aNewData |= p2->Data();
            nValues = core_Population (aNewMask, aNewData);
            break;
          }
          p2 = p2->Next();
        }
        // Store the block - result of operation
        if (Resizable()) {
          ReSize (myNbPackedMapNodes);
        }
        const int aHashCode = HashCode (aKeyInt, myNbBuckets);
        myData1[aHashCode] = new t_intMapNode (aNewMask, aNewData,
                                                     myData1[aHashCode]);
        ++myNbPackedMapNodes;
        myExtent += nValues;
        p1 = p1->Next();
      }
    }
    // Iteration of the 2nd map.
    for (int i = 0; i <= nBuckets2; i++)
    {
      const t_intMapNode* p2 = theMap2.myData1[i];
      while (p2 != 0L)
      {
        // Find aKey - the base address of currently iterated block
        const int aKey = p2->Key();
        const int aKeyInt = packedKeyIndex (aKey);
        // Find the corresponding block in the 1st map
        const t_intMapNode* p1 = theMap1.myData1[HashCode (aKeyInt, nBuckets1)];
        while (p1)
        {
          if (p1->IsEqual(aKeyInt))
            break;
          p1 = p1->Next();
        }
        // Add the block from the 2nd map only in the case when the similar
        // block has not been found in the 1st map
        if (p1 == 0L)
        {
          if (Resizable())
          {
            ReSize (myNbPackedMapNodes);
          }
          const int aHashCode = HashCode (aKeyInt, myNbBuckets);
          myData1[aHashCode]= new t_intMapNode (p2->Mask(), p2->Data(),
                                                      myData1[aHashCode]);
          ++myNbPackedMapNodes;
          myExtent += p2->NbValues();
        }
        p2 = p2->Next();
      }
    }
  }
}

//=======================================================================
//function : Unite
//purpose  : Boolean operation OR with the given map
//=======================================================================

bool core_PackedMapOfInteger::Unite(const core_PackedMapOfInteger& theMap)
{
  if (theMap.IsEmpty() || myData1 == theMap.myData1) // A | 0 == A | A == A
    return false;
  else if ( IsEmpty() ) { // 0 | B == B
    Assign ( theMap );
    return true;
  }

  size_t aNewExtent (myExtent);
  const int nBuckets2 = theMap.myNbBuckets;

  // Iteration of the 2nd map.
  for (int i = 0; i <= nBuckets2; i++)
  {
    const t_intMapNode* p2 = theMap.myData1[i];
    while (p2 != 0L)
    {
      // Find aKey - the base address of currently iterated block of integers
      const int aKey = p2->Key();
      const int aKeyInt = packedKeyIndex (aKey);
      // Find the corresponding block in the 1st (this) map
      int aHashCode = HashCode (aKeyInt, myNbBuckets);
      t_intMapNode* p1 = myData1[aHashCode];
      while (p1)
      {
        if (p1->IsEqual(aKeyInt))
        {
          const size_t anOldPop = p1->NbValues();
          unsigned int newData = p1->Data() | p2->Data();
          if ( newData != p1->Data() ) {
            p1->ChangeData() = newData;
            aNewExtent = aNewExtent - anOldPop +
                          core_Population (p1->ChangeMask(), newData);
          }
          break;
        }
        p1 = p1->Next();
      }
      // If the block is not found in the 1st map, add it to the 1st map
      if (p1 == 0L)
      {
        if (Resizable())
        {
          ReSize (myNbPackedMapNodes);
          aHashCode = HashCode (aKeyInt, myNbBuckets);
        }
        myData1[aHashCode] = new t_intMapNode (p2->Mask(), p2->Data(),
                                                     myData1[aHashCode]);
        ++myNbPackedMapNodes;
        aNewExtent += p2->NbValues();
      }
      p2 = p2->Next();
    }
  }
  bool isChanged = ( myExtent != aNewExtent );
  myExtent = aNewExtent;
  return isChanged;
}

//=======================================================================
//function : Intersection
//purpose  : Boolean operation AND between 2 maps
//=======================================================================

void core_PackedMapOfInteger::Intersection
                                (const core_PackedMapOfInteger& theMap1,
                                 const core_PackedMapOfInteger& theMap2)
{
  if (theMap1.IsEmpty() || theMap2.IsEmpty()) // A & 0 == 0 & B == 0
    Clear();
  else if (myData1 == theMap1.myData1)
    Intersect (theMap2);
  else if (myData1 == theMap2.myData1)
    Intersect (theMap1);
  else {
    const t_intMapNode* const* aData1;
    const t_intMapNode* const* aData2;
    int nBuckets1, nBuckets2;
    if (theMap1.Extent() < theMap2.Extent())
    {
      aData1 = theMap1.myData1;
      aData2 = theMap2.myData1;
      nBuckets1 = theMap1.myNbBuckets;
      nBuckets2 = theMap2.myNbBuckets;
    }
    else
    {
      aData1 = theMap2.myData1;
      aData2 = theMap1.myData1;
      nBuckets1 = theMap2.myNbBuckets;
      nBuckets2 = theMap1.myNbBuckets;
    }
    Clear();

    // Iteration of the 1st map.
    for (int i = 0; i <= nBuckets1; i++)
    {
      const t_intMapNode* p1 = aData1[i];
      while (p1 != 0L)
      {
        // Find aKey - the base address of currently iterated block
        const int aKey = p1->Key();
        const int aKeyInt = packedKeyIndex (aKey);
        // Find the corresponding block in the 2nd map
        const t_intMapNode* p2 = aData2[HashCode(aKeyInt, nBuckets2)];
        while (p2)
        {
          if (p2->IsEqual(aKeyInt))
          {
            const unsigned int aNewData = p1->Data() & p2->Data();
            // Store the block - result of operation
            if (aNewData)
            {
              if (Resizable())
              {
                ReSize (myNbPackedMapNodes);
              }
              const int aHashCode = HashCode (aKeyInt, myNbBuckets);
              unsigned int aNewMask = p1->Mask();
              myExtent += core_Population (aNewMask, aNewData);
              myData1[aHashCode]= new t_intMapNode(aNewMask, aNewData,
                                                         myData1[aHashCode]);
              ++myNbPackedMapNodes;
            }
            break;
          }
          p2 = p2->Next();
        }
        p1 = p1->Next();
      }
    }
  }
}

//=======================================================================
//function : Intersect
//purpose  : Boolean operation AND with the given map
//=======================================================================

bool core_PackedMapOfInteger::Intersect
                 (const core_PackedMapOfInteger& theMap)
{
  if ( IsEmpty() ) // 0 & B == 0
    return false;
  else if (theMap.IsEmpty()) { // A & 0 == 0
    Clear();
    return true;
  }
  else if (myData1 == theMap.myData1) // A & A == A
    return false;

  size_t aNewExtent (0);
  const int nBuckets2 = theMap.myNbBuckets;

  // Iteration of this map.
  for (int i = 0; i <= myNbBuckets; i++)
  {
    t_intMapNode* q  = 0L;
    t_intMapNode* p1 = myData1[i];
    while (p1 != 0L)
    {
      // Find aKey - the base address of currently iterated block of integers
      const int aKey = p1->Key();
      const int aKeyInt = packedKeyIndex (aKey);
      // Find the corresponding block in the 2nd map
      const t_intMapNode* p2 = theMap.myData1[HashCode (aKeyInt, nBuckets2)];
      while (p2)
      {
        if (p2->IsEqual(aKeyInt))
        {
          const unsigned int aNewData = p1->Data() & p2->Data();
          // Store the block - result of operation
          if (aNewData == 0)
            p2 = 0L;  // no match - the block has to be removed
          else
          {
            if ( aNewData != p1->Data() )
              p1->ChangeData() = aNewData;
            aNewExtent += core_Population (p1->ChangeMask(), aNewData);
          }
          break;
        }
        p2 = p2->Next();
      }
      t_intMapNode* pNext = p1->Next();
      // If p2!=NULL, then the map node is kept and we move to the next one
      // Otherwise we should remove the current node
      if (p2)
      {
        q = p1;
      }
      else
      {
        --myNbPackedMapNodes;
        if (q)  q->SetNext (pNext);
        else    myData1[i]  = pNext;
        delete p1;
      }
      p1 = pNext;
    }
  }
  bool isChanged = ( myExtent != aNewExtent );
  myExtent = aNewExtent;
  return isChanged;
}

//=======================================================================
//function : Subtraction
//purpose  : Boolean operation SUBTRACT between two maps
//=======================================================================

void core_PackedMapOfInteger::Subtraction
                                (const core_PackedMapOfInteger& theMap1,
                                 const core_PackedMapOfInteger& theMap2)
{
  if (theMap1.IsEmpty() || theMap2.myData1 == theMap1.myData1) // 0 \ A == A \ A == 0
    Clear();
  else if (theMap2.IsEmpty()) // A \ 0 == A
    Assign (theMap1);
  else if (myData1 == theMap1.myData1)
    Subtract (theMap2);
  else if (myData1 == theMap2.myData1) {
    core_PackedMapOfInteger aMap;
    aMap.Subtraction ( theMap1, theMap2 );
    Assign ( aMap );
  }
  else {
    const int nBuckets1 = theMap1.myNbBuckets;
    const int nBuckets2 = theMap2.myNbBuckets;
    Clear();

    // Iteration of the 1st map.
    for (int i = 0; i <= nBuckets1; i++)
    {
      const t_intMapNode * p1 = theMap1.myData1[i];
      while (p1 != 0L)
      {
        // Find aKey - the base address of currently iterated block of integers
        const int aKey = p1->Key();
        const int aKeyInt = packedKeyIndex (aKey);
        unsigned int aNewMask = p1->Mask();
        unsigned int aNewData = p1->Data();
        size_t       nValues (p1->NbValues());
        // Find the corresponding block in the 2nd map
        const t_intMapNode* p2 = theMap2.myData1[HashCode (aKeyInt, nBuckets2)];
        while (p2)
        {
          if (p2->IsEqual(aKeyInt))
          {
            aNewData &= ~p2->Data();
            nValues = core_Population (aNewMask, aNewData);
            break;
          }
          p2 = p2->Next();
        }
        // Store the block - result of operation
        if (aNewData)
        {
          if (Resizable())
          {
            ReSize (myNbPackedMapNodes);
          }
          const int aHashCode = HashCode (aKeyInt, myNbBuckets);
          myData1[aHashCode]= new t_intMapNode (aNewMask, aNewData,
                                                      myData1[aHashCode]);
          ++myNbPackedMapNodes;
          myExtent += nValues;
        }
        p1 = p1->Next();
      }
    }
  }
}

//=======================================================================
//function : Subtract
//purpose  : Boolean operation SUBTRACT with the given map
//=======================================================================

bool core_PackedMapOfInteger::Subtract
                                (const core_PackedMapOfInteger& theMap)
{
  if ( IsEmpty() || theMap.IsEmpty() ) // 0 \ B == 0; A \ 0 == A
    return false;
  else if (myData1 == theMap.myData1) { // A \ A == 0
    Clear();
    return true;
  }
  else {
    size_t aNewExtent (0);
    const int nBuckets2 = theMap.myNbBuckets;
    // Iteration of this map.
    for (int i = 0; i <= myNbBuckets; i++)
    {
      t_intMapNode* q  = 0L;
      t_intMapNode* p1 = myData1[i];
      while (p1 != 0L)
      {
        // Find aKey - the base address of currently iterated block of integers
        const int aKey = p1->Key();
        const int aKeyInt = packedKeyIndex (aKey);
        t_intMapNode* pNext = p1->Next();
        // Find the corresponding block in the 2nd map
        const t_intMapNode* p2 = theMap.myData1[HashCode (aKeyInt, nBuckets2)];
        while (p2)
        {
          if (p2->IsEqual(aKeyInt))
          {
            const unsigned int aNewData = p1->Data() & ~p2->Data();
            // Store the block - result of operation
            if (aNewData == 0)
            {
              // no match - the block has to be removed
              --myNbPackedMapNodes;
              if (q)  q->SetNext (pNext);
              else    myData1[i]  = pNext;
              delete p1;
            }
            else if ( aNewData != p1->Data() )
            {
              p1->ChangeData() = aNewData;
              aNewExtent += core_Population (p1->ChangeMask(), aNewData);
              q = p1;
            }
            else
            {
              aNewExtent += p1->NbValues();
              q = p1;
            }
            break;
          }
          p2 = p2->Next();
        }
        if (p2 == 0L)
        {
          aNewExtent += p1->NbValues();
          q = p1;
        }
        p1 = pNext;
      }
    }
    bool isChanged = ( myExtent != aNewExtent );
    myExtent = aNewExtent;
    return isChanged;
  }
}

//=======================================================================
//function : Difference
//purpose  : Boolean operation XOR 
//=======================================================================

void core_PackedMapOfInteger::Difference  (const core_PackedMapOfInteger& theMap1,
                                              const core_PackedMapOfInteger& theMap2)
{
  if (theMap1.IsEmpty()) // 0 ^ B == B
    Assign (theMap2);
  else if (theMap2.IsEmpty()) // A ^ 0 == A
    Assign (theMap1);
  else if (myData1 == theMap1.myData1)
    Differ(theMap2);
  else if (myData1 == theMap2.myData1)
    Differ(theMap1);
  else {
    int i;
    const int nBuckets1 = theMap1.myNbBuckets;
    const int nBuckets2 = theMap2.myNbBuckets;
    Clear();

    // Iteration of the 1st map.
    for (i = 0; i <= nBuckets1; i++)
    {
      const t_intMapNode* p1 = theMap1.myData1[i];
      while (p1 != 0L)
      {
        // Find aKey - the base address of currently iterated block of integers
        const int aKey = p1->Key();
        const int aKeyInt = packedKeyIndex (aKey);
        unsigned int aNewMask = p1->Mask();
        unsigned int aNewData = p1->Data();
        size_t       nValues (p1->NbValues());
        // Find the corresponding block in the 2nd map
        const t_intMapNode* p2 = theMap2.myData1[HashCode (aKeyInt, nBuckets2)];
        while (p2)
        {
          if (p2->IsEqual(aKeyInt))
          {
            aNewData ^= p2->Data();
            nValues = core_Population (aNewMask, aNewData);
            break;
          }
          p2 = p2->Next();
        }
        // Store the block - result of operation
        if (aNewData)
        {
          if (Resizable())
          {
            ReSize (myNbPackedMapNodes);
          }
          const int aHashCode = HashCode (aKeyInt, myNbBuckets);
          myData1[aHashCode]= new t_intMapNode (aNewMask, aNewData,
                                                      myData1[aHashCode]);
          ++myNbPackedMapNodes;
          myExtent += nValues;
        }
        p1 = p1->Next();
      }
    }
    
    // Iteration of the 2nd map.
    for (i = 0; i <= nBuckets2; i++)
    {
      const t_intMapNode* p2 = theMap2.myData1[i];
      while (p2 != 0L)
      {
        // Find aKey - the base address of currently iterated block
        const int aKey = p2->Key();
        const int aKeyInt = packedKeyIndex (aKey);
        // Find the corresponding block in the 1st map
        const t_intMapNode* p1 = theMap1.myData1[HashCode (aKeyInt, nBuckets1)];
        while (p1)
        {
          if (p1->IsEqual(aKeyInt))
            break;
          p1 = p1->Next();
        }
        // Add the block from the 2nd map only in the case when the similar
        // block has not been found in the 1st map
        if (p1 == 0L)
        {
          if (Resizable())
          {
            ReSize (myNbPackedMapNodes);
          }
          const int aHashCode = HashCode (aKeyInt, myNbBuckets);
          myData1[aHashCode]= new t_intMapNode (p2->Mask(), p2->Data(),
                                                      myData1[aHashCode]);
          ++myNbPackedMapNodes;
          myExtent += p2->NbValues();
        }
        p2 = p2->Next();
      }
    }
  }
}

//=======================================================================
//function : Differ
//purpose  : Boolean operation XOR 
//=======================================================================
  
bool core_PackedMapOfInteger::Differ(const core_PackedMapOfInteger& theMap)
{
  if (theMap.IsEmpty()) // A ^ 0 = A
    return false;    
  else if (IsEmpty()) { // 0 ^ B = B
    Assign ( theMap );
    return true;
  }
  else if( myData1 == theMap.myData1) { // A ^ A == 0
    Clear();
    return true;
  }

  size_t aNewExtent (0);
  const int nBuckets2 = theMap.myNbBuckets;
  bool isChanged = false;
  // Iteration by other map
  for (int i = 0; i <= nBuckets2; i++)
  {
      t_intMapNode * q  = 0L;
    const t_intMapNode* p2 = theMap.myData1[i];
    while (p2 != 0L)
    {
      // Find aKey - the base address of currently iterated block
      const int aKey = p2->Key();
      const int aKeyInt = packedKeyIndex (aKey);
        
      // Find the corresponding block in the 1st map
      t_intMapNode* p1 = myData1[HashCode (aKeyInt, myNbBuckets)];
      t_intMapNode* pNext = p1->Next();
      while (p1)
      {
        if (p1->IsEqual(aKeyInt))
        {
          const unsigned int aNewData = p1->Data() ^ p2->Data();
          // Store the block - result of operation
          if (aNewData == 0)
          {
            // no match - the block has to be removed
            --myNbPackedMapNodes;
            if (q)  q->SetNext (pNext);
            else    myData1[i]  = pNext;
            delete p1;
          }
          else if ( aNewData != p1->Data() )
          {
            p1->ChangeData() = aNewData;
            isChanged = true;
            aNewExtent += core_Population (p1->ChangeMask(), aNewData);
            q = p1;
          }
          break;
        }
        p1 = pNext;
      }
      // Add the block from the 2nd map only in the case when the similar
      // block has not been found in the 1st map
      if (p1 == 0L)
      {
        if (Resizable())
        {
          ReSize (myNbPackedMapNodes);
        }
        const int aHashCode = HashCode (aKeyInt, myNbBuckets);
        myData1[aHashCode] = new t_intMapNode (p2->Mask(), p2->Data(),
                                                     myData1[aHashCode]);
        ++myNbPackedMapNodes;
        aNewExtent += p2->NbValues();
        isChanged = true;
      }
      p2 = p2->Next();
    }
  }
  myExtent = aNewExtent;
  return isChanged;
}

//=======================================================================
//function : IsEqual
//purpose  : Boolean operation returns true if this map is equal to the other map  
//=======================================================================

bool core_PackedMapOfInteger::IsEqual(const core_PackedMapOfInteger& theMap) const
{
  if (IsEmpty() && theMap.IsEmpty())
    return true;
  else if ( Extent() != theMap.Extent() )
    return false;
  else if(myData1 == theMap.myData1)
    return true;

  const int nBuckets2 = theMap.myNbBuckets;
  // Iteration of this map.
  for (int i = 0; i <= myNbBuckets; i++)
  {
    const t_intMapNode* p1 = myData1[i];
    while (p1 != 0L)
    {
      // Find aKey - the base address of currently iterated block of integers
      const int aKey = p1->Key();
      const int aKeyInt = packedKeyIndex (aKey);
      t_intMapNode* pNext = p1->Next();
      // Find the corresponding block in the 2nd map
      const t_intMapNode* p2 = theMap.myData1[HashCode (aKeyInt, nBuckets2)];
      while (p2)
      {
        if ( p2->IsEqual(aKeyInt) )
        {
          if ( p1->Data() != p2->Data() )
            return false;
          break;
        }
        p2 = p2->Next();
      }
      // if the same block not found, maps are different
      if (p2 == 0L)
        return false;

      p1 = pNext;
    }
  }
  return true;
}

//=======================================================================
//function : IsSubset
//purpose  : Boolean operation returns true if this map if subset of other map
//=======================================================================

bool core_PackedMapOfInteger::IsSubset (const core_PackedMapOfInteger& theMap) const
{
  if ( IsEmpty() ) // 0 <= A 
    return true;
  else if ( theMap.IsEmpty() ) // ! ( A <= 0 )
    return false;
  else if ( Extent() > theMap.Extent() )
    return false;
  else if(myData1 == theMap.myData1)
    return true;

  const int nBuckets2 = theMap.myNbBuckets;
  // Iteration of this map.
  for (int i = 0; i <= myNbBuckets; i++)
  {
    const t_intMapNode* p1 = myData1[i];
    while (p1 != 0L)
    {
      // Find aKey - the base address of currently iterated block of integers
      const int aKey = p1->Key();
      const int aKeyInt = packedKeyIndex (aKey);
      t_intMapNode* pNext = p1->Next();
      // Find the corresponding block in the 2nd map
      const t_intMapNode* p2 = theMap.myData1[HashCode (aKeyInt, nBuckets2)];
      if (!p2)
        return false;
      while (p2)
      {
        if ( p2->IsEqual(aKeyInt) )
        {
          if ( p1->Data() & ~p2->Data() ) // at least one bit set in p1 is not set in p2
            return false;
          break;
        }
        p2 = p2->Next();
      }
      p1 = pNext;
    }
  }
  return true;
}

//=======================================================================
//function : HasIntersection
//purpose  : Boolean operation returns true if this map intersects with other map
//=======================================================================

bool core_PackedMapOfInteger::HasIntersection (const core_PackedMapOfInteger& theMap) const
{
  if (IsEmpty() || theMap.IsEmpty()) // A * 0 == 0 * B == 0
    return false;

  if(myData1 == theMap.myData1)
    return true;

  const int nBuckets2 = theMap.myNbBuckets;
  // Iteration of this map.
  for (int i = 0; i <= myNbBuckets; i++)
  {
    const t_intMapNode* p1 = myData1[i];
    while (p1 != 0L) {
      // Find aKey - the base address of currently iterated block of integers
      const int aKey = p1->Key();
      const int aKeyInt = packedKeyIndex (aKey);
      t_intMapNode* pNext = p1->Next();
      // Find the corresponding block in the 2nd map
      const t_intMapNode* p2 = theMap.myData1[HashCode (aKeyInt, nBuckets2)];
      while (p2)
      {
        if (p2->IsEqual(aKeyInt))
        {
          if (p1->Data() & p2->Data())
            return true;
          break;
        }
        p2 = p2->Next();
      }
      p1 = pNext;
    }
  }
  return false;
}
