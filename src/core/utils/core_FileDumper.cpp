//-----------------------------------------------------------------------------
// Created on: 04 November 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

// Own include
#include <mobius/core_FileDumper.h>

//-----------------------------------------------------------------------------
// Used definitions
//-----------------------------------------------------------------------------

#define QRDUMP_DELIMIT_LINE \
  "===================================================================================================="

#define QRDUMP_DELIMIT_LINE_SHORT1 \
  "    **************************************************************************************"

#define QRDUMP_DELIMIT_LINE_SHORT2 \
  "    -------------------------------------------------------------"

#define QRDUMP_TAB                "\t"
#define QRDUMP_NL                 "\n"
#define QRDUMP_COMMA              ","
#define QRDUMP_SPACE              " "
#define QRDUMP_LBRACKET           "["
#define QRDUMP_RBRACKET           "]"
#define QRDUMP_TRUE               "true"
#define QRDUMP_FALSE              "false"
#define QRDUMP_REALPART           "Re"
#define QRDUMP_IMAGPART           "Im"
#define QRDUMP_PREFIX_TYPE_INT    "{Integer}"
#define QRDUMP_PREFIX_TYPE_REAL   "{Real}"
#define QRDUMP_PREFIX_TYPE_BOOL   "{Boolean}"
#define QRDUMP_PREFIX_TYPE_ARRAY  "{Array}"
#define QRDUMP_PREFIX_TYPE_MATRIX "{Matrix}"

//-----------------------------------------------------------------------------
// Construction
//-----------------------------------------------------------------------------

//! Default constructor.
mobius::core_FileDumper::core_FileDumper()
{
}

//! Constructor opening file for dumping.
//! \param filename [in] file to dump data to.
mobius::core_FileDumper::core_FileDumper(const std::string& filename)
{
  this->Open(filename);
}

//! Destructor releasing the acquired resources.
mobius::core_FileDumper::~core_FileDumper()
{
  if ( m_FILE.is_open() )
    m_FILE.close();
}

//-----------------------------------------------------------------------------
// Dumping
//-----------------------------------------------------------------------------

//! Opens file with the passed filename for dumping.
//! \param filename [in] file to open.
//! \return true in case of success, false -- otherwise.
bool mobius::core_FileDumper::Open(const std::string& filename)
{
  m_FILE.open(filename.c_str(), std::ios::out | std::ios::trunc);
  return m_FILE.is_open();
}

//! Dumps the passed message to the target file.
//! \param msg [in] message to dump.
void mobius::core_FileDumper::Dump(const std::string& msg)
{
  if ( !m_FILE.is_open() )
    return;

  m_FILE << msg.c_str() << QRDUMP_NL << QRDUMP_NL;
}

//! Dumps the passed scalar value to the target file.
//! \param val [in] data to dump.
//! \param msg [in] optional message.
void mobius::core_FileDumper::Dump(const int val,
                                   const std::string& msg)
{
  if ( !m_FILE.is_open() )
    return;

  if ( msg.length() )
    m_FILE << QRDUMP_LBRACKET << msg.c_str() << QRDUMP_RBRACKET << QRDUMP_SPACE;

  m_FILE << QRDUMP_PREFIX_TYPE_INT << QRDUMP_NL;
  m_FILE << val;
  m_FILE << QRDUMP_NL << QRDUMP_NL;
}

//! Dumps the passed scalar value to the target file.
//! \param val [in] data to dump.
//! \param msg [in] optional message.
void mobius::core_FileDumper::Dump(const double val,
                                   const std::string& msg)
{
  if ( !m_FILE.is_open() )
    return;

  if ( msg.length() )
    m_FILE << QRDUMP_LBRACKET << msg.c_str() << QRDUMP_RBRACKET << QRDUMP_SPACE;

  m_FILE << QRDUMP_PREFIX_TYPE_REAL << QRDUMP_NL;
  m_FILE << val;
  m_FILE << QRDUMP_NL << QRDUMP_NL;
}

//! Dumps the passed Boolean value to the target file.
//! \param val [in] data to dump.
//! \param msg [in] optional message.
void mobius::core_FileDumper::Dump(const bool val,
                                   const std::string& msg)
{
  if ( !m_FILE.is_open() )
    return;

  if ( msg.length() )
    m_FILE << QRDUMP_LBRACKET << msg.c_str() << QRDUMP_RBRACKET << QRDUMP_SPACE;

  m_FILE << QRDUMP_PREFIX_TYPE_BOOL << QRDUMP_NL;
  m_FILE << (val ? QRDUMP_TRUE : QRDUMP_FALSE);
  m_FILE << QRDUMP_NL << QRDUMP_NL;
}

//! Dumps the passed array of real values to the target file.
//! \param arr [in] data to dump.
//! \param numElems [in] number of elements in the array.
//! \param msg [in] optional message.
void mobius::core_FileDumper::Dump(const double* arr,
                                   const int numElems,
                                   const std::string& msg)
{
  if ( !m_FILE.is_open() )
    return;

  if ( !numElems )
    return;

  if ( msg.length() )
    m_FILE << QRDUMP_LBRACKET << msg.c_str() << QRDUMP_RBRACKET << QRDUMP_SPACE;

  m_FILE << QRDUMP_PREFIX_TYPE_ARRAY << QRDUMP_NL;

  for ( int i = 0; i < numElems; ++i )
  {
    m_FILE << arr[i];
    if ( i < numElems - 1 )
      m_FILE << QRDUMP_TAB;
  }

  m_FILE << QRDUMP_NL << QRDUMP_NL;
}

//! Dumps the passed array of real values as matrix to the target file.
//! \param mx [in] data to dump.
//! \param numRows [in] number of rows.
//! \param numCols [in] number of columns.
//! \param msg [in] optional message.
void mobius::core_FileDumper::Dump(const double* mx,
                                   const int numRows,
                                   const int numCols,
                                   const std::string& msg)
{
  if ( !m_FILE.is_open() )
    return;

  if ( !numRows || !numCols )
    return;

  if ( msg.length() )
    m_FILE << QRDUMP_LBRACKET << msg.c_str() << QRDUMP_RBRACKET << QRDUMP_SPACE;

  m_FILE << QRDUMP_PREFIX_TYPE_MATRIX << QRDUMP_NL;

  for ( int i = 0; i < numRows; ++i )
  {
    for ( int j = 0; j < numCols; ++j )
    {
      m_FILE << mx[i*numCols + j];
      if ( (i + j + 2) < (numRows + numCols) )
        m_FILE << QRDUMP_TAB;
    }
    m_FILE << QRDUMP_NL;
  }

  m_FILE << QRDUMP_NL;
}
