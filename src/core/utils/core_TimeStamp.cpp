//-----------------------------------------------------------------------------
// Created on: 07 December 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

#include <mobius/core_TimeStamp.h>

// Windows includes
#include <windows.h>

//! Generates timestamp structure for the current time.
//! \return timestamp structure.
mobius::core_Ptr<mobius::core_TimeStamp> mobius::core_TimeStampTool::Generate()
{
  time_t t = -1;
  time(&t);

  int internalCount = 0;

  // TODO: Windows ONLY (!!!)
  static LONG INTERNAL = 0;
  internalCount = (int) InterlockedIncrement(&INTERNAL);

  return new core_TimeStamp(t, internalCount);
}

//! Converts the passed timestamp structure to the corresponding array of
//! integer values.
//! \param TS [in] timestamp structure to convert.
//! \return correspondent array of integer data chunks.
std::vector<int>
  mobius::core_TimeStampTool::AsChunked(const core_Ptr<core_TimeStamp>& TS)
{
  std::vector<int> res;
  if ( TS->Time != -1 )
  {
    tm timeInfo;
    localtime_s(&timeInfo, &TS->Time);

    res.push_back(timeInfo.tm_sec);
    res.push_back(timeInfo.tm_min);
    res.push_back(timeInfo.tm_hour);
    res.push_back(timeInfo.tm_mday);
    res.push_back(timeInfo.tm_mon);
    res.push_back(timeInfo.tm_year);
    res.push_back(timeInfo.tm_wday);
    res.push_back(timeInfo.tm_yday);
    res.push_back(timeInfo.tm_isdst);
    res.push_back(TS->Internal);
  }
  else
  {
    for ( int i = 0; i < 10; ++i )
      res.push_back(-1);
  }

  return res;
}

//! Converts the passed array of integer data chunks to TimeStamp structure.
//! \param chunked [in] input array.
//! \return correspondent timestamp structure.
mobius::core_Ptr<mobius::core_TimeStamp>
  mobius::core_TimeStampTool::FromChunked(const std::vector<int>& chunked)
{
  if ( chunked.size() != 10 )
    return new core_TimeStamp();

  tm timeInfo;
  timeInfo.tm_sec   = chunked.at(0);
  timeInfo.tm_min   = chunked.at(1);
  timeInfo.tm_hour  = chunked.at(2);
  timeInfo.tm_mday  = chunked.at(3);
  timeInfo.tm_mon   = chunked.at(4);
  timeInfo.tm_year  = chunked.at(5);
  timeInfo.tm_wday  = chunked.at(6);
  timeInfo.tm_yday  = chunked.at(7);
  timeInfo.tm_isdst = chunked.at(8);
  time_t t = mktime(&timeInfo);

  return new core_TimeStamp( t, chunked.at(9) );
}
