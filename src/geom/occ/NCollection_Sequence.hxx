// Created on: 2002-03-28
// Created by: Alexander GRIGORIEV
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

#ifndef NCollection_Sequence_HeaderFile
#define NCollection_Sequence_HeaderFile

#include <mobius/NCollection_BaseSequence.hxx>
#include <mobius/NCollection_StlIterator.hxx>
#include <mobius/Standard_OutOfRange.hxx>
#include <mobius/Standard_NoSuchObject.hxx>

namespace mobius {
namespace occ {

/**
 * Purpose:     Definition of a sequence of elements indexed by
 *              an Integer in range of 1..n
 */              
template <class TheItemType>
class NCollection_Sequence : public NCollection_BaseSequence
{
public:
  //! STL-compliant typedef for value type
  typedef TheItemType value_type;

public:
  //!   Class defining sequence node - for internal use by Sequence
  class Node : public NCollection_SeqNode
  {
  public:
    //! Constructor
    Node (const TheItemType& theItem) :
      NCollection_SeqNode ()
      { myValue = theItem; }
    //! Constant value access
    const TheItemType& Value () const { return myValue; }
    //! Variable value access
    TheItemType&       ChangeValue () { return myValue; }

  private:
    TheItemType    myValue;
  }; // End of nested class Node

 public:
  //!   Implementation of the Iterator interface.
  class Iterator : public NCollection_BaseSequence::Iterator
  {
  public:
    //! Empty constructor - for later Init
    Iterator  (void) {}
    //! Constructor with initialisation
    Iterator  (const NCollection_Sequence& theSeq,
               const bool      isStart = Standard_True)
      : NCollection_BaseSequence::Iterator (theSeq, isStart) {}
    //! Check end
    bool More (void) const
    { return (myCurrent!=NULL); }
    //! Make step
    void Next (void)
    {
      if (myCurrent)
      {
        myPrevious = myCurrent;
        myCurrent = myCurrent->Next();
      }
    }
    //! Constant value access
    const TheItemType& Value (void) const
    { return ((const Node *)myCurrent)->Value(); }
    //! Variable value access
    TheItemType& ChangeValue (void) const
    { return ((Node *)myCurrent)->ChangeValue(); }
    //! Performs comparison of two iterators.
    bool IsEqual (const Iterator& theOther) const
    {
      return myCurrent == theOther.myCurrent;
    }
  }; // End of nested class Iterator

  //! Shorthand for a regular iterator type.
  typedef NCollection_StlIterator<std::bidirectional_iterator_tag, Iterator, TheItemType, false> iterator;

  //! Shorthand for a constant iterator type.
  typedef NCollection_StlIterator<std::bidirectional_iterator_tag, Iterator, TheItemType, true> const_iterator;

  //! Returns an iterator pointing to the first element in the sequence.
  iterator begin() const { return Iterator (*this, true); }

  //! Returns an iterator referring to the past-the-end element in the sequence.
  iterator end() const { Iterator anIter (*this, false); anIter.Next(); return anIter; }
  
  //! Returns a const iterator pointing to the first element in the sequence.
  const_iterator cbegin() const { return Iterator (*this, true); }

  //! Returns a const iterator referring to the past-the-end element in the sequence.
  const_iterator cend() const { Iterator anIter (*this, false); anIter.Next(); return anIter; }

 public:
  // ---------- PUBLIC METHODS ------------

  //! Empty constructor.
  NCollection_Sequence() : NCollection_BaseSequence(Handle(NCollection_BaseAllocator)()) {}

  //! Constructor
  explicit NCollection_Sequence (const Handle(NCollection_BaseAllocator)& theAllocator) : NCollection_BaseSequence(theAllocator) {}

  //! Copy constructor
  NCollection_Sequence (const NCollection_Sequence& theOther) :
    NCollection_BaseSequence(theOther.myAllocator)
  {
    Assign (theOther);
  }

  //! Number of items
  int Size (void) const
  { return mySize; }

  //! Number of items
  int Length (void) const
  { return mySize; }

  //! Method for consistency with other collections.
  //! @return Lower bound (inclusive) for iteration.
  int Lower() const
  {
    return 1;
  }

  //! Method for consistency with other collections.
  //! @return Upper bound (inclusive) for iteration.
  int Upper() const
  {
    return mySize;
  }

  //! Empty query
  bool IsEmpty (void) const
  { return (mySize==0); }

  //! Reverse sequence
  void Reverse (void)
  { PReverse(); }

  //! Exchange two members
  void Exchange (const int I,
                 const int J )
  { PExchange(I, J); }

  //! Static deleter to be passed to BaseSequence
  static void delNode (NCollection_SeqNode * theNode, 
                       Handle(NCollection_BaseAllocator)& theAl)
  {
    ((Node *) theNode)->~Node();
    theAl->Free(theNode);
  }

  //! Clear the items out, take a new allocator if non null
  void Clear (const Handle(NCollection_BaseAllocator)& theAllocator=0L)
  {
    ClearSeq (delNode);
    if (!theAllocator.IsNull())
      this->myAllocator = theAllocator;
  }
  
  //! Replace this sequence by the items of theOther.
  //! This method does not change the internal allocator.
  NCollection_Sequence& Assign (const NCollection_Sequence& theOther)
  { 
    if (this != &theOther)
    {
      Clear();
      appendSeq((const Node *)theOther.myFirstItem);
    }
    return * this;
  }

  //! Replacement operator
  NCollection_Sequence& operator= (const NCollection_Sequence& theOther)
  {
    return Assign (theOther);
  }

  //! Remove one item
  void Remove (Iterator& thePosition)
  { RemoveSeq (thePosition, delNode); }

  //! Remove one item
  void Remove (const int theIndex)
  { RemoveSeq (theIndex, delNode); }

  //! Remove range of items
  void Remove (const int theFromIndex,
               const int theToIndex)
  { RemoveSeq (theFromIndex, theToIndex, delNode); }

  //! Append one item
  void Append (const TheItemType& theItem)
  { PAppend (new (this->myAllocator) Node (theItem)); }

  //! Append another sequence (making it empty)
  void Append (NCollection_Sequence& theSeq)
  {
    if (this == &theSeq || theSeq.IsEmpty())
      return;
    if (this->myAllocator == theSeq.myAllocator)
    {
      // Then we take the sequence and glue it to our end - 
      // deallocation will bring no problem
      PAppend(theSeq);
    }
    else
    {
      // No - this sequence has different memory scope
      appendSeq((const Node *)theSeq.myFirstItem);
      theSeq.Clear();
    }
  }

  //! Prepend one item
  void Prepend (const TheItemType& theItem)
  { PPrepend (new (this->myAllocator) Node (theItem)); }

  //! Prepend another sequence (making it empty)
  void Prepend (NCollection_Sequence& theSeq)
  {
    if (this == &theSeq || theSeq.IsEmpty())
      return;
    if (this->myAllocator == theSeq.myAllocator)
    {
      // Then we take the sequence and glue it to our head - 
      // deallocation will bring no problem
      PPrepend(theSeq);
    }
    else
    {
      // No - this sequence has different memory scope
      prependSeq((const Node *)theSeq.myFirstItem, 1);
      theSeq.Clear();
    }
  }

  //! InsertBefore theIndex theItem
  void InsertBefore (const int theIndex, 
                     const TheItemType&     theItem)
  { InsertAfter (theIndex-1, theItem); }

  //! InsertBefore theIndex another sequence (making it empty)
  void InsertBefore (const int theIndex,
                     NCollection_Sequence&  theSeq)
  { InsertAfter (theIndex-1, theSeq); }
  
  //! InsertAfter the position of iterator
  void InsertAfter  (Iterator&              thePosition,
                     const TheItemType&     theItem)
  { PInsertAfter (thePosition, new (this->myAllocator) Node (theItem)); }

  //! InsertAfter theIndex another sequence (making it empty)
  void InsertAfter  (const int theIndex,
                     NCollection_Sequence&  theSeq)
  {
    if (this == &theSeq || theSeq.IsEmpty())
      return;
    if (this->myAllocator == theSeq.myAllocator)
    {
      // Then we take the list and glue it to our head - 
      // deallocation will bring no problem
      PInsertAfter(theIndex, theSeq);
    }
    else
    {
      // No - this sequence has different memory scope
      prependSeq((const Node *)theSeq.myFirstItem, theIndex + 1);
      theSeq.Clear();
    }
  }

  //! InsertAfter theIndex theItem
  void InsertAfter (const int  theIndex, 
                    const TheItemType&      theItem)
  {
    Standard_OutOfRange_Raise_if (theIndex < 0 || theIndex > mySize, "NCollection_Sequence::InsertAfter");
    PInsertAfter (theIndex, new (this->myAllocator) Node (theItem));
  }

  //! Split in two sequences
  void Split (const int theIndex, NCollection_Sequence& theSeq)
  {
    theSeq.Clear (this->myAllocator);
    PSplit (theIndex, theSeq);
  }

  //! First item access
  const TheItemType& First () const
  {
    Standard_NoSuchObject_Raise_if (mySize == 0, "NCollection_Sequence::First");
    return ((const Node *) myFirstItem) -> Value();
  }

  //! First item access
  TheItemType& ChangeFirst()
  {
    Standard_NoSuchObject_Raise_if (mySize == 0, "NCollection_Sequence::ChangeFirst");
    return ((Node* )myFirstItem)->ChangeValue();
  }

  //! Last item access
  const TheItemType& Last () const
  {
    Standard_NoSuchObject_Raise_if (mySize == 0, "NCollection_Sequence::Last");
    return ((const Node *) myLastItem) -> Value();
  }

  //! Last item access
  TheItemType& ChangeLast()
  {
    Standard_NoSuchObject_Raise_if (mySize == 0, "NCollection_Sequence::ChangeLast");
    return ((Node* )myLastItem)->ChangeValue();
  }

  //! Constant item access by theIndex
  const TheItemType& Value (const int theIndex) const
  {
    Standard_OutOfRange_Raise_if (theIndex <= 0 || theIndex > mySize, "NCollection_Sequence::Value");

    NCollection_Sequence * const aLocalTHIS = (NCollection_Sequence *) this;
    aLocalTHIS -> myCurrentItem  = Find (theIndex);
    aLocalTHIS -> myCurrentIndex = theIndex;
    return ((const Node *) myCurrentItem) -> Value();
  }

  //! Constant operator()
  const TheItemType& operator() (const int theIndex) const
  { return Value(theIndex); }

  //! Variable item access by theIndex
  TheItemType& ChangeValue (const int theIndex)
  {
    Standard_OutOfRange_Raise_if (theIndex <= 0 || theIndex > mySize, "NCollection_Sequence::ChangeValue");

    myCurrentItem  = Find (theIndex);
    myCurrentIndex = theIndex;
    return ((Node *) myCurrentItem) -> ChangeValue();
  }

  //! Variable operator()
  TheItemType& operator() (const int theIndex)
  { return ChangeValue(theIndex); }

  //! Set item value by theIndex
  void SetValue (const int theIndex, 
                 const TheItemType& theItem)
  { ChangeValue (theIndex) = theItem; }

  // ******** Destructor - clears the Sequence
  virtual ~NCollection_Sequence (void)
  { Clear(); }

 private:

  // ---------- FRIEND CLASSES ------------
  friend class Iterator;

  // ----------- PRIVATE METHODS -----------

  //! append the sequence headed by the given Node
  void appendSeq(const Node * pCur)
  {
    while (pCur)
    {
      Node* pNew = new (this->myAllocator) Node(pCur->Value());
      PAppend(pNew);
      pCur = (const Node *)pCur->Next();
    }
  }

  //! insert the sequence headed by the given Node before the item with the given index
  void prependSeq(const Node * pCur, int ind)
  {
    ind--;
    while (pCur)
    {
      Node* pNew = new (this->myAllocator) Node(pCur->Value());
      PInsertAfter(ind++, pNew);
      pCur = (const Node *)pCur->Next();
    }
  }

};

}
}

#endif
