//-----------------------------------------------------------------------------
// Created on: 15 January 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

#ifndef QrTest_Quaternion_HeaderFile
#define QrTest_Quaternion_HeaderFile

// QrTest includes
#include <QrTest_CaseIDs.h>

// QrTestLib includes
#include <QrTestLib_TestCase.h>

// QrCore includes
#include <QrCore_Types.h>

//! Unit test for quaternions.
class QrTest_Quaternion : public QrTestLib_TestCase
{
public:

  //! Returns Test Case ID.
  //! \return ID of the Test Case.
  static int ID()
  {
    return CaseID_Core_Quaternion;
  }

  //! Returns filename for the description.
  //! \return filename for the description of the Test Case.
  static std::string DescriptionFn()
  {
    return "QrTest_Quaternion";
  }

  //! Returns Test Case description directory.
  //! \return description directory for the Test Case.
  static std::string DescriptionDir()
  {
    return "Core";
  }

  //! Returns pointers to the Test Functions to launch.
  //! \param functions [out] output collection of pointers.
  static void Functions(QrTestFunctions& functions)
  {
    functions << &create
              << &add
              << &subtract
              << &product_qn
              << &product_scalar
              << &dot_product
              << &cross_product
              << &invert
              << &conjugate
              << &to_matrix;
  }

private:

  static bool create        (const int funcID);
  static bool add           (const int funcID);
  static bool subtract      (const int funcID);
  static bool product_qn    (const int funcID);
  static bool product_scalar(const int funcID);
  static bool dot_product   (const int funcID);
  static bool cross_product (const int funcID);
  static bool invert        (const int funcID);
  static bool conjugate     (const int funcID);
  static bool to_matrix     (const int funcID);

};

#endif
