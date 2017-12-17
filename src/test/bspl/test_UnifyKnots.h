//-----------------------------------------------------------------------------
// Created on: 06 March 2015
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Quaoar library. It has no license protection. You
// are free to use it the way you like, including incorporating of Quaoar
// sources and libraries into commercial applications.
// Web: http://quaoar.su
//-----------------------------------------------------------------------------

#ifndef QrTest_BasisUnifyKnots_HeaderFile
#define QrTest_BasisUnifyKnots_HeaderFile

// QrTest includes
#include <QrTest_CaseIDs.h>

// QrTestLib includes
#include <QrTestLib_TestCase.h>

// QrCore includes
#include <QrCore_Types.h>

//! Unit test for knots unification procedure.
class QrTest_BasisUnifyKnots : public QrTestLib_TestCase
{
public:

  //! Returns Test Case ID.
  //! \return ID of the Test Case.
  static int ID()
  {
    return CaseID_BSpl_Basis_BasisUnifyKnots;
  }

  //! Returns filename for the description.
  //! \return filename for the description of the Test Case.
  static std::string DescriptionFn()
  {
    return "QrTest_BasisUnifyKnots";
  }

  //! Returns Test Case description directory.
  //! \return description directory for the Test Case.
  static std::string DescriptionDir()
  {
    return "BSpl";
  }

  //! Returns pointers to the Test Functions to launch.
  //! \param functions [out] output collection of pointers.
  static void Functions(QrTestFunctions& functions)
  {
    functions << &test_unify;
  }

private:

  static bool test_unify(const int funcID);

};

#endif
