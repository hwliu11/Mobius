// Created on: 2002-04-18
// Created by: Alexander KARTOMIN (akm)
// Copyright (c) 2002-2014 OPEN CASCADE SAS
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

// Purpose:     Implementation of the BaseMap class

#include <mobius/NCollection_BaseMap.hxx>

#include <mobius/Standard_OutOfRange.hxx>

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
static const Standard_Integer THE_TCollection_Primes[THE_NB_PRIMES] =
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

//=======================================================================
//function : BeginResize
//purpose  : 
//=======================================================================

Standard_Boolean  NCollection_BaseMap::BeginResize
  (const int  NbBuckets,
   int&       N,
   NCollection_ListNode**& data1,
   NCollection_ListNode**& data2) const 
{
  // get next size for the buckets array
  N = NextPrimeForMap(NbBuckets);
  if (N <= myNbBuckets)
  {
    if (!myData1)
      N = myNbBuckets;
    else
      return Standard_False;
  }
  data1 = (NCollection_ListNode **)
    myAllocator->Allocate((N+1)*sizeof(NCollection_ListNode *));
  memset(data1, 0, (N+1)*sizeof(NCollection_ListNode *));
  if (isDouble) 
  {
    data2 = (NCollection_ListNode **)
      myAllocator->Allocate((N+1)*sizeof(NCollection_ListNode *));
    memset(data2, 0, (N+1)*sizeof(NCollection_ListNode *));
  }
  else
    data2 = NULL;
  return Standard_True;
}

//=======================================================================
//function : EndResize
//purpose  : 
//=======================================================================

void  NCollection_BaseMap::EndResize
  (const int theNbBuckets,
   const int N,
   NCollection_ListNode** data1,
   NCollection_ListNode** data2)
{
  (void )theNbBuckets; // obsolete parameter
  if (myData1) 
    myAllocator->Free(myData1);
  if (myData2) 
    myAllocator->Free(myData2);
  myNbBuckets = N;
  myData1 = data1;
  myData2 = data2;
}


//=======================================================================
//function : Destroy
//purpose  : 
//=======================================================================

void  NCollection_BaseMap::Destroy (NCollection_DelMapNode fDel,
                                    Standard_Boolean doReleaseMemory)
{
  if (!IsEmpty()) 
  {
    int i;
    NCollection_ListNode** data = (NCollection_ListNode**) myData1;
    NCollection_ListNode *p,*q;
    for (i = 0; i <= NbBuckets(); i++) 
    {
      if (data[i]) 
      {
        p = data[i];
        while (p) 
        {
          q = (NCollection_ListNode*)p->Next();
          fDel (p, myAllocator);
          p = q;
        }
        data[i] = NULL;
      }
    }
  }

  mySize = 0;
  if (doReleaseMemory)
  {
    if (myData1) 
      myAllocator->Free(myData1);
    if (isDouble && myData2) 
      myAllocator->Free(myData2);
    myData1 = myData2 = NULL;
  }
}


//=======================================================================
//function : Statistics
//purpose  : 
//=======================================================================

void NCollection_BaseMap::Statistics(std::ostream& S) const
{
  S <<"\nMap Statistics\n---------------\n\n";
  S <<"This Map has "<<myNbBuckets<<" Buckets and "<<mySize<<" Keys\n\n";
  
  if (mySize == 0) return;

  // compute statistics on 1
  int * sizes = new int [mySize+1];
  int i,l,nb;
  NCollection_ListNode* p;
  NCollection_ListNode** data;
  
  S << "\nStatistics for the first Key\n";
  for (i = 0; i <= mySize; i++) sizes[i] = 0;
  data = (NCollection_ListNode **) myData1;
  nb = 0;
  for (i = 0; i <= myNbBuckets; i++) 
  {
    l = 0;
    p = data[i];
    if (p) nb++;
    while (p) 
    {
      l++;
      p = p->Next();
    }
    sizes[l]++;
  }

  // display results
  l = 0;
  for (i = 0; i<= mySize; i++) 
  {
    if (sizes[i] > 0) 
    {
      l += sizes[i] * i;
      S << std::setw(5) << sizes[i] <<" buckets of size "<<i<<"\n";
    }
  }

  double mean = ((double) l) / ((double) nb);
  S<<"\n\nMean of length : "<<mean<<"\n";

  delete [] sizes;
}

//=======================================================================
//function : NextPrimeForMap
//purpose  : 
//=======================================================================

int NCollection_BaseMap::NextPrimeForMap
  (const int N) const
{
  for (Standard_Integer aPrimeIter = 0; aPrimeIter < THE_NB_PRIMES; ++aPrimeIter)
  {
    if (THE_TCollection_Primes[aPrimeIter] > N)
    {
      return THE_TCollection_Primes[aPrimeIter];
    }
  }
  throw Standard_OutOfRange ("TCollection::NextPrimeForMap() - requested too big size");
}

