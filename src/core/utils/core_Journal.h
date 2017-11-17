//-----------------------------------------------------------------------------
// Created on: 15 September 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
//-----------------------------------------------------------------------------

#ifndef core_Journal_HeaderFile
#define core_Journal_HeaderFile

// core includes
#include <mobius/core_JournalTicket.h>

namespace mobius {

// Forward declaration for friending.
class core_JournalSentry;

//! \ingroup MOBIUS_CORE
//!
//! Journal for background algorithmic recording. The journaling routine is
//! thread-safe and partially concurrent. You are free to use its logging
//! capabilities from different threads, however, from time to time their
//! execution will be serialized. Serialization happens due to
//! the fact that Journal Tickets are stored in blocks grouped into
//! non-concurrent list. Each time this list has to be grown,
//! a mutex is acquired.
//!
//! \note Definitely, it is possible to use some concurrent collections (e.g.
//!       TBB) but we do not do this to improve reusability of journaling
//!       tools.
//!
//! Basically you can think of Journal as a series of ASCII string lines
//! thrown by the algorithm and appearing in arbitrary order. We expect
//! Journal to contain the same collection of records for
//! any number of threads. Therefore, if you obtain different records,
//! this is an unambiguous signal that you have data races in your algorithm.
//!
//! The arbitrary order of records complicates comparison of Journals.
//! Thus we recommend to compare them with care (e.g. only in maintenance
//! routines or during nightly tests). If you enable journaling in a
//! single-threaded context, you can use line-per-line comparison as the
//! order of Tickets is deterministic for sequential algorithms. In such
//! cases you can benefit from dumping Journals to files and compare the files
//! instead of Journals. However, the latter is mostly a matter of sense.
class core_Journal : public OBJECT
{
friend class core_JournalSentry;

protected:

  //! Internal structure for a block of Tickets.
  struct block
  {
    //! Default ctor.
    block() : pTickets(NULL), pNext(NULL) {}

    //! Constructor with ticket.
    //! \param tickets [in] pointer to the first ticket.
    block(core_JournalTicket* tickets) : pTickets(tickets), pNext(NULL) {}

    //! Constructor with ticket and next block.
    //! \param tickets [in] pointer to the first ticket.
    //! \param next    [in] pointer to the next block.
    block(core_JournalTicket* tickets, block* next) : pTickets(tickets), pNext(next) {}

    core_JournalTicket *pTickets; //!< Pointer to the first ticket.
    block              *pNext;    //!< Pointer to the next block.
  };

// Class-level API:
public:

  //! Create an instance of a journal.
  //! \return journal instance.
  inline static core_Ptr<core_Journal> Instance()
  {
    return new core_Journal();
  }

public:

  mobiusCore_EXPORT
    core_Journal(const size_t block_size = 512);

  mobiusCore_EXPORT
    ~core_Journal();

public:

  mobiusCore_EXPORT void
    Dump(std::ostream& out) const;

  mobiusCore_EXPORT core_Journal&
    operator<<(const core_JournalTicket& ticket);

  mobiusCore_EXPORT core_StringBuffer
    ToBuffer(const bool onlyNonEmpty = false) const;

  mobiusCore_EXPORT bool
    Save(const std::string& filename) const;

  mobiusCore_EXPORT bool
    Restore(const std::string& filename);

  mobiusCore_EXPORT int
    NumOfTickets() const;

public:

  //! Checks whether this Journal contains the same Tickets as the passed
  //! one. Tickets are compared by their content, so this method is very
  //! slow -- O(n^2). Its usage out of unit testing scope is discouraged.
  //! \param other [in] Journal to compare with.
  //! \param compare [in] comparator.
  //! \return true in case of equality, false -- otherwise.
  template<typename TComparator>
  bool
    CompareByContent( const core_Ptr<core_Journal>& other,
                      const TComparator& compare = core_FltTicketCompare(1.0e-3, 1.0e-3) )
  {
    // Compare by number of Tickets first
    if ( this->NumOfTickets() != other->NumOfTickets() )
      return false;

    // Set of already matched Tickets
    std::set<int> matched;

    // Loop over this Journal
    block* this_next = m_pFirstBlock;
    while ( this_next )
    {
      for ( size_t this_t = 0; this_t < m_blockSize; ++this_t )
      {
        core_JournalTicket this_ticket( this_next->pTickets[this_t] );
        bool is_this_ticked_found = false;

        // Now loop over the passed Journal attempting to find the same Ticket
        block* other_next = other->m_pFirstBlock;
        int other_block_idx = 0;
        while ( other_next )
        {
          for ( size_t other_t = 0; other_t < other->m_blockSize; ++other_t )
          {
            const int other_ticket_id = (int) (other_block_idx*other->m_blockSize + other_t);
            if ( matched.find(other_ticket_id) != matched.end() )
              continue;

            core_JournalTicket other_ticket( other_next->pTickets[other_t] );

            // Compare both Tickets
            if ( compare(this_ticket, other_ticket) )
            {
              matched.insert(other_ticket_id);
              is_this_ticked_found = true;
              break;
            }
          }

          if ( is_this_ticked_found )
            break;

          // Next block
          other_next = other_next->pNext;
          other_block_idx++;
        }

        if ( !is_this_ticked_found )
          return false;
      }

      // Next block
      this_next = this_next->pNext;
    }

    return true;
  }

private:

  mobiusCore_EXPORT void
    addTicket(const core_JournalTicket& ticket);

  mobiusCore_EXPORT void
    index(const size_t ticket_ID,
          size_t& block_index,
          size_t& index_in_block) const;

private:

  core_Journal(const core_Journal&) {}
  void operator=(const core_Journal&) {}

// Changed in one thread
private:

  //! Blocks of Tickets (pointer to the first one).
  block* m_pFirstBlock;

  //! Block size.
  size_t m_blockSize;

  //! Critical section barrier for threads in order to prevent them
  //! from allocation of blocks array simultaneously.
  CRITICAL_SECTION m_mutex;

// Changed in multiple threads
private:

  //! Number of blocks.
  size_t m_iNumBlocks;

  //! Blocks of Tickets (pointer to the last one).
  block* m_pLastBlock;

  //! Internal counter for Ticket IDs. Each Journal uses its own
  //! zero-based numeration of Tickets. Note that this counter is
  //! incremented by atomic operation.
  LONG m_iINTERNAL;

};

//! \ingroup MOBIUS_CORE
//!
//! Safe entry to Journal.
class core_JournalSentry
{
public:

  //! Default constructor.
  core_JournalSentry(const int = 0) {}

  //! Constructor accepting journal instance.
  //! \param Jrn [in] journal instance.
  core_JournalSentry(const core_Ptr<core_Journal>& Jrn) : m_journal(Jrn) {}

public:

  //! Dumps the journal contents to the passed output stream.
  //! \param out [in/out] target output stream.
  void Dump(std::ostream& out) const
  {
    if ( !m_journal.IsNull() )
      m_journal->Dump(out);
  }

  //! Adds the passed ticked to the Journal.
  //! \param ticket [in] ticket to add.
  //! \return this instance for convenience.
  core_JournalSentry& operator<<(const core_JournalTicket& ticket)
  {
    if ( !m_journal.IsNull() )
      m_journal->addTicket(ticket);

    return *this;
  }

  //! \return Journal instance.
  const core_Ptr<core_Journal>& Jrn() const { return m_journal; }

private:

  core_Ptr<core_Journal> m_journal; //!< Journal instance.

};

};

#endif
