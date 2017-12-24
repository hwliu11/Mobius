//-----------------------------------------------------------------------------
// Created on: 15 September 2014
//-----------------------------------------------------------------------------
// Copyright (c) 2017, Sergey Slyadnev
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
#include <mobius/core_Journal.h>

// STD includes
#include <fstream>

//! Constructor accepting block size.
//! \param block_size [in] block size.
mobius::core_Journal::core_Journal(const size_t block_size)
: OBJECT(),
  m_blockSize(block_size),
  m_iNumBlocks(1),
  m_iINTERNAL(-1)
{
  m_pFirstBlock = new block( new core_JournalTicket[m_blockSize] );
  m_pLastBlock = m_pFirstBlock;

  // Initialize mutex
  InitializeCriticalSection(&m_mutex);
}

//! Destructor.
mobius::core_Journal::~core_Journal()
{
  DeleteCriticalSection(&m_mutex);

  block* next = m_pFirstBlock;
  while ( next )
  {
    block* current = next;
    next = current->pNext;
    delete[] current->pTickets;
    delete current;
  }
}

//! Dumps all the contents of the journal to the passed output stream.
//! \param out [in/out] target output stream.
void mobius::core_Journal::Dump(std::ostream& out) const
{
  // Loop over the Tickets
  size_t ticket_id = 0;
  block* next = m_pFirstBlock;
  while ( next )
  {
    core_JournalTicket* tickets = next->pTickets;
    if ( tickets )
    {
      for ( size_t t = 0; t < m_blockSize; ++t )
      {
        out << (int) (ticket_id++) << ": " << tickets[t].ToString() << "\n";
      }
    }

    // Move to next block
    next = next->pNext;
  }
}

//! Adds the passed ticket to the Journal.
//! \param ticket [in] ticket to add.
//! \return this instance for convenience.
mobius::core_Journal& mobius::core_Journal::operator<<(const core_JournalTicket& ticket)
{
  this->addTicket(ticket);
  return *this;
}

//! Dumps the entire Journal contents to buffer.
//! \param onlyNonEmpty [in] if true, empty tickets are skipped.
//! \return buffer.
mobius::core_StringBuffer mobius::core_Journal::ToBuffer(const bool onlyNonEmpty) const
{
  core_StringBuffer result;

  // Loop over the Tickets
  size_t ticket_id = 0;
  block* next = m_pFirstBlock;
  while ( next )
  {
    core_JournalTicket* tickets = next->pTickets;
    if ( tickets )
    {
      for ( size_t t = 0; t < m_blockSize; ++t )
      {
        if ( onlyNonEmpty && tickets[t].Msg == "" )
          continue;

        result << (int) (ticket_id++) << ": " << tickets[t].ToString() << "\n";
      }
    }

    // Move to next block
    next = next->pNext;
  }

  return result;
}

//! Saves Journal to file with the given name.
//! \param filename [in] destination file.
//! \return true in case of success, false -- otherwise.
bool mobius::core_Journal::Save(const std::string& filename) const
{
  // Open file for dumping
  FILE* file_ptr = NULL;
  fopen_s(&file_ptr, filename.c_str(), "w");

  if ( !file_ptr )
    return false;

  fprintf_s( file_ptr, this->ToBuffer().String.c_str() );

  // Close file
  fclose(file_ptr);
  return true;
}

//! Restores Journal from file with the given name.
//! \param filename [in] source file.
//! \return true in case of success, false -- otherwise.
bool mobius::core_Journal::Restore(const std::string& filename)
{
  std::ifstream in_stream;
  in_stream.open( filename.c_str() );

  if ( !in_stream.is_open() )
    return false;

  // Read file
  std::string line;
  while ( !in_stream.eof() )
  {
    in_stream >> line;

    // Split by label delimiter
    std::vector<std::string> chunks;
    core::split(line, ":", chunks);

    // Remove prefix
    core::join(chunks, line, 1);

    // Add line to Journal
    if ( line.length() )
      this->addTicket( core_JournalTicket(line) );
  }

  // Close file
  in_stream.close();
  return true;
}

//! Returns the number of Tickets.
//! \return number of tickets.
int mobius::core_Journal::NumOfTickets() const
{
  if ( !m_iNumBlocks )
    return 0;

  const int num_completely_occupied = (int) ( (m_iNumBlocks - 1)*m_blockSize );
  int num_in_last_block = 0;

  // Loop over lar block (may be not completely occupied)
  for ( size_t t = 0; t < m_blockSize; ++t )
  {
    core_JournalTicket ticket( m_pLastBlock->pTickets[t] );
    if ( !ticket.Msg.empty() )
      num_in_last_block++;
    else
      break;
  }

  return num_completely_occupied + num_in_last_block;
}

//! Adds the passed Ticket to the Journal.
//! \param ticket [in] Ticket to add.
void mobius::core_Journal::addTicket(const core_JournalTicket& ticket)
{
  // Global counting ID for each ticket is associated right at
  // construction time
  const size_t ID = (size_t) InterlockedIncrement(&m_iINTERNAL);

  // Get block index
  size_t block_index, index_in_block;
  this->index(ID, block_index, index_in_block);

  EnterCriticalSection(&m_mutex); // Acquire mutex
  { // Notice that if(...) must be in a critical section: otherwise several threads may pass

    // Check if memory should be grown
    if ( block_index >= m_iNumBlocks )
    {
      // New block for Tickets
      block* prevLast = m_pLastBlock ? m_pLastBlock : m_pFirstBlock;
      m_pLastBlock = new block( new core_JournalTicket[m_blockSize] );
      prevLast->pNext = m_pLastBlock;

      // One more block
      m_iNumBlocks++;
    }
  }
  LeaveCriticalSection(&m_mutex); // Release mutex

  // Find target block
  block* target = m_pFirstBlock;
  size_t next_idx = 0;
  while ( next_idx++ < block_index )
    target = target->pNext;

  // Insert
  target->pTickets[index_in_block] = ticket.Msg;
}

//! Returns block index by ID of the Ticket.
//! \param ticket_ID [in] ticket ID.
//! \param block_index [out] block index.
//! \param index_in_block [out] relative index of Ticket in block.
void mobius::core_Journal::index(const size_t ticket_ID,
                                 size_t&      block_index,
                                 size_t&      index_in_block) const
{
  block_index    = ticket_ID / m_blockSize;
  index_in_block = ticket_ID % m_blockSize;
}
