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

#ifndef NCollection_BaseMap_HeaderFile
#define NCollection_BaseMap_HeaderFile

#include <mobius/Standard.hxx>
#include <mobius/NCollection_BaseAllocator.hxx>
#include <mobius/NCollection_DefineAlloc.hxx>
#include <mobius/NCollection_ListNode.hxx>

namespace mobius {
namespace occ {

typedef void (* NCollection_DelMapNode)
     (NCollection_ListNode*, Handle(NCollection_BaseAllocator)& theAl);

/**
 * Purpose:     This is a base class for all Maps:
 *                Map
 *                DataMap
 *                DoubleMap
 *                IndexedMap
 *                IndexedDataMap
 *              Provides utilitites for managing the buckets.
 */              

class NCollection_BaseMap 
{
public:
  //! Memory allocation
  DEFINE_STANDARD_ALLOC
  DEFINE_NCOLLECTION_ALLOC

public:
  // **************************************** Class Iterator ****************
  class Iterator
  {
  protected:
    //! Empty constructor
    Iterator (void) :
      myNbBuckets (0),
      myBuckets   (NULL),
      myBucket    (0),
      myNode      (NULL) {}
    
    //! Constructor
    Iterator (const NCollection_BaseMap& theMap) :
      myNbBuckets (theMap.myNbBuckets),
      myBuckets   (theMap.myData1),
      myBucket    (-1),
      myNode      (NULL)
    {
      if (!myBuckets) 
        myNbBuckets = -1;
      else
        do {
          myBucket++;
          if (myBucket > myNbBuckets) 
            return;
          myNode = myBuckets[myBucket];
        } while (!myNode);
    }

  public:
    //! Initialize
    void Initialize (const NCollection_BaseMap& theMap)
    {
      myNbBuckets = theMap.myNbBuckets;
      myBuckets = theMap.myData1;
      myBucket = -1;
      myNode = NULL;
      if (!myBuckets) 
        myNbBuckets = -1;
      PNext();
    }

    //! Reset
    void Reset (void)
    {
      myBucket = -1;
      myNode = NULL;
      PNext();
    }
    
    //! Performs comparison of two iterators.
    Standard_Boolean IsEqual (const Iterator& theOther) const
    {
      return myBucket == theOther.myBucket && myNode == theOther.myNode;
    }

  protected:
    //! PMore
    Standard_Boolean PMore (void) const
    { return (myNode != NULL); }
    
    //! PNext
    void PNext (void)
    {
      if (!myBuckets) 
        return; 
      if (myNode) 
      {
        myNode = myNode->Next();
        if (myNode) 
          return;
      }
      while (!myNode) 
      {
        myBucket++;
        if (myBucket > myNbBuckets) 
          return;
        myNode = myBuckets[myBucket];
      }
    }

  protected:
    // ---------- PRIVATE FIELDS ------------
    int       myNbBuckets; //!< Total buckets in the map
    NCollection_ListNode **myBuckets;   //!< Location in memory 
    int       myBucket;    //!< Current bucket
    NCollection_ListNode * myNode;      //!< Current node
  };

 public:
  // ---------- PUBLIC METHODS ------------

  //! NbBuckets
  int NbBuckets() const
  { return myNbBuckets; }

  //! Extent
  int Extent() const
  { return mySize; }

  //! IsEmpty
  Standard_Boolean IsEmpty() const
  { return mySize == 0; }

  //! Statistics
  mobiusGeom_EXPORT void Statistics(std::ostream& S) const;

  //! Returns attached allocator
  const Handle(NCollection_BaseAllocator)& Allocator() const
  { return myAllocator; }

 protected:
  // -------- PROTECTED METHODS -----------

  //! Constructor
  NCollection_BaseMap (const int NbBuckets,
                       const Standard_Boolean single,
                       const Handle(NCollection_BaseAllocator)& theAllocator)
  : myData1(NULL),
    myData2(NULL),
    myNbBuckets(NbBuckets),
    mySize(0),
    isDouble(!single)
  {
    myAllocator = (theAllocator.IsNull() ? NCollection_BaseAllocator::CommonBaseAllocator() : theAllocator);
  }

  //! Destructor
  virtual ~NCollection_BaseMap() {}

  //! BeginResize
  mobiusGeom_EXPORT Standard_Boolean BeginResize 
    (const int  NbBuckets,
     int&       NewBuckets,
     NCollection_ListNode**& data1,
     NCollection_ListNode**& data2) const;

  //! EndResize
  mobiusGeom_EXPORT void EndResize 
    (const int NbBuckets,
     const int NewBuckets,
     NCollection_ListNode** data1,
     NCollection_ListNode** data2);

  //! Resizable
  Standard_Boolean Resizable() const
  { return IsEmpty() || (mySize > myNbBuckets); }

  //! Increment
  int Increment() { return ++mySize; }

  //! Decrement
  int Decrement()  { return --mySize; }

  //! Destroy
  mobiusGeom_EXPORT void Destroy(NCollection_DelMapNode fDel,
                               Standard_Boolean doReleaseMemory = Standard_True);

  //! NextPrimeForMap
  mobiusGeom_EXPORT int NextPrimeForMap
    (const int N) const;

  //! Exchange content of two maps without data copying
  void exchangeMapsData (NCollection_BaseMap& theOther)
  {
    std::swap (myAllocator, theOther.myAllocator);
    std::swap (myData1,     theOther.myData1);
    std::swap (myData2,     theOther.myData2);
    std::swap (myNbBuckets, theOther.myNbBuckets);
    std::swap (mySize,      theOther.mySize);
    //std::swap (isDouble,    theOther.isDouble);
  }

 protected:
  // --------- PROTECTED FIELDS -----------
  Handle(NCollection_BaseAllocator) myAllocator;
  NCollection_ListNode ** myData1;
  NCollection_ListNode ** myData2;

 private: 
  // ---------- PRIVATE FIELDS ------------
  int myNbBuckets;
  int mySize;
  Standard_Boolean isDouble;

  // ---------- FRIEND CLASSES ------------
  friend class Iterator;
};

}
}

#endif
