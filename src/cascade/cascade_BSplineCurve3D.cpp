//-----------------------------------------------------------------------------
// Created on: 05 August 2013
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
#include <mobius/cascade_BSplineCurve3D.h>

// Cascade includes
#include <mobius/cascade_MultResolver.h>

// OCCT includes
#include <AdvApprox_ApproxAFunction.hxx>
#include <AdvApprox_EvaluatorFunction.hxx>
#include <gp_Pnt.hxx>
#include <NCollection_Sequence.hxx>
#include <Standard_ProgramError.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_HArray1OfInteger.hxx>
#include <TColStd_HArray1OfReal.hxx>

//-----------------------------------------------------------------------------
// Evaluator for OCCT Advanced Approximation
//-----------------------------------------------------------------------------

namespace mobius {

//! Auxiliary class for evaluation of points and derivatives on the nested
//! curve. This class plays as a data source for approximation algorithm.
//! It provides possibility to obtain curve data for any value within the
//! specified parametric range.
class cascade_BSplineCurve3D_Eval : public AdvApprox_EvaluatorFunction
{
public:

  //! Constructs the Evaluator's instance passing the source curve
  //! and the parametric range.
  //! \param theCurve [in] source curve to evaluate.
  //! \param theFirstU [in] first value of the parametric range.
  //! \param theLastU [in] second value of the parametric range.
  cascade_BSplineCurve3D_Eval(const Ptr<bcurve>& theCurve,
                              const double theFirstU,
                              const double theLastU)
  {
    m_curve = theCurve;
    m_range[0] = theFirstU;
    m_range[1] = theLastU;
  }

  virtual void Evaluate(int* theDimension,
                        double theRange[2],
                        double* theParam,
                        int* theOrder,
                        double* theResult,
                        int* theErrorCode);

private:

  //! Curve to evaluate.
  Ptr<bcurve> m_curve;

  //! Parametric range.
  Standard_Real m_range[2];

};

//! Evaluates the curve. Invoked by approximation algorithm.
//! \param theDimension [in] spatial dimension.
//! \param theRange [in] parametric range.
//! \param theParam [in] parameter value to evaluate the curve for.
//! \param theOrder [in] derivation order to evaluate (C0, C1 or C2).
//! \param theResult [out] evaluation result.
//! \param theErrorCode [out] code of the error occured (if any).
void cascade_BSplineCurve3D_Eval::Evaluate(int* theDimension,
                                           double theRange[2],
                                           double* theParam,
                                           int* theOrder,
                                           double* theResult,
                                           int* theErrorCode)
{
  *theErrorCode = 0; // Reset evaluation error code
  double u = *theParam;

  if ( *theDimension != 3 )
    *theErrorCode = 1;

  // Re-initialize range if the requested one differs from the stored one
  if ( theRange[0] != m_range[0] || theRange[1] != m_range[1] )
  {
    m_range[0] = theRange[0];
    m_range[1] = theRange[1];
  }

  // Resulting values
  xyz PointOnCurve;

  // Answer the query
  switch ( *theOrder )
  {
  case 0:
    m_curve->Eval(u, PointOnCurve);
    theResult[0] = PointOnCurve.X();
    theResult[1] = PointOnCurve.Y();
    theResult[2] = PointOnCurve.Z();
    break;
  case 1:
  case 2:
    Standard_ProgramError::Raise("Not yet implemented");
  default:
    theResult[0] = theResult[1] = theResult[2] = 0.0;
    *theErrorCode = 3;
    break;
  }
}

};

//-----------------------------------------------------------------------------
// Mobius-OCCT connector
//-----------------------------------------------------------------------------

//! Constructor.
//! \param mobiusCurve [in] Mobius 3D curve to convert.
mobius::cascade_BSplineCurve3D::cascade_BSplineCurve3D(const Ptr<bcurve>& mobiusCurve)
{
  m_mobiusCurve = mobiusCurve;
  m_fMaxError   = 0.0;
  m_bIsDone     = false;
}

//-----------------------------------------------------------------------------

//! Constructor.
//! \param occtCurve [in] OCCT 3D curve to convert.
mobius::cascade_BSplineCurve3D::cascade_BSplineCurve3D(const Handle(Geom_BSplineCurve)& occtCurve)
{
  m_occtCurve = occtCurve;
  m_fMaxError = 0.0;
  m_bIsDone   = false;
}

//-----------------------------------------------------------------------------

//! Destructor.
mobius::cascade_BSplineCurve3D::~cascade_BSplineCurve3D()
{}

//-----------------------------------------------------------------------------

//! Converts Mobius B-spline curve to OCCT one via re-approximation.
//! \param theTol3d       [in] tolerance to achieve.
//! \param theOrder       [in] desired order.
//! \param theMaxSegments [in] maximum number of segments.
//! \param theMaxDegree   [in] maximum degree.
void mobius::cascade_BSplineCurve3D::ReApproxMobius(const double        theTol3d,
                                                    const GeomAbs_Shape theOrder,
                                                    const int           theMaxSegments,
                                                    const int           theMaxDegree)
{
  m_fMaxError = 0.0;

  // Working variables
  int num1DSS = 0, num2DSS = 0, num3DSS = 1;
  Handle(TColStd_HArray1OfReal) oneDTolNul, twoDTolNul;
  Handle(TColStd_HArray1OfReal) threeDTol = new TColStd_HArray1OfReal(1, num3DSS);
  threeDTol->Init(theTol3d);

  // Access parametric range
  double f = m_mobiusCurve->MinParameter();
  double l = m_mobiusCurve->MaxParameter();

  // Re-approximate curve using OCCT Advanced Approximation facilities
  cascade_BSplineCurve3D_Eval Eval(m_mobiusCurve, f, l);
  AdvApprox_ApproxAFunction Approx(num1DSS, num2DSS, num3DSS,
                                   oneDTolNul, twoDTolNul, threeDTol,
                                   f, l,
                                   theOrder,
                                   theMaxDegree, theMaxSegments,
                                   Eval);

  m_bIsDone = Approx.IsDone() && Approx.HasResult();

  // Create OCCT NURBS curve in case of success
  if ( m_bIsDone )
  {
    TColgp_Array1OfPnt poles( 1, Approx.NbPoles() );
    Approx.Poles(1, poles);
    Handle(TColStd_HArray1OfReal) knots = Approx.Knots();
    Handle(TColStd_HArray1OfInteger) mults = Approx.Multiplicities();
    const int degree = Approx.Degree();
    m_occtCurve = new Geom_BSplineCurve(poles, knots->Array1(), mults->Array1(), degree);
    m_fMaxError = Approx.MaxError(3, 1);
  }
}

//-----------------------------------------------------------------------------

//! Converts Mobius B-spline curve to OCCT one or vice versa by direct
//! supplying of knots, multiplicities and poles as they are.
void mobius::cascade_BSplineCurve3D::DirectConvert()
{
  if ( !m_mobiusCurve.IsNull() )
    this->convertToOpenCascade();
  else if ( !m_occtCurve.IsNull() )
    this->convertToMobius();
}

//-----------------------------------------------------------------------------

//! Accessor for the Mobius curve.
//! \return Mobius curve.
const mobius::Ptr<mobius::bcurve>&
  mobius::cascade_BSplineCurve3D::GetMobiusCurve() const
{
  return m_mobiusCurve;
}

//-----------------------------------------------------------------------------

//! Accessor for the OpenCascade curve.
//! \return OpenCascade curve.
const Handle(Geom_BSplineCurve)&
  mobius::cascade_BSplineCurve3D::GetOpenCascadeCurve() const
{
  return m_occtCurve;
}

//-----------------------------------------------------------------------------

//! Returns true if the result is accessible, false -- otherwise.
//! \return true/false.
bool mobius::cascade_BSplineCurve3D::IsDone() const
{
  return m_bIsDone;
}

//-----------------------------------------------------------------------------

//! Returns maximum achieved approximation error.
//! \return maximum error.
double mobius::cascade_BSplineCurve3D::MaxError() const
{
  return m_fMaxError;
}

//-----------------------------------------------------------------------------

void mobius::cascade_BSplineCurve3D::convertToOpenCascade()
{
  const std::vector<xyz>& srcPoles = m_mobiusCurve->Poles();
  std::vector<double>     srcU     = m_mobiusCurve->Knots();
  const int               srcDeg   = m_mobiusCurve->Degree();

  // Poles are transferred as-is.
  TColgp_Array1OfPnt occtPoles( 1, (int) srcPoles.size() );
  for ( int i = occtPoles.Lower(); i <= occtPoles.Upper(); ++i )
  {
    gp_Pnt P( srcPoles[i - 1].X(), srcPoles[i - 1].Y(), srcPoles[i - 1].Z() );
    occtPoles(i) = P;
  }

  // The way how knots are processed in Mobius is different from OCCT.
  // OCCT stores each knot value just once, however, requires additional
  // array with multiplicities. E.g. U = (0, 0, 1, 2, 2) is represented by
  // two arrays in OCCT: (0, 1, 2) and (2, 1, 2). Mobius is more straightforward
  // concerning this. MultResolver tool performs necessary conversion from
  // Mobius to OCCT style.
  cascade_MultResolver MResolver;
  for ( int i = 0; i < (int) srcU.size(); ++i )
  {
    MResolver.Resolve(srcU[i]);
  }

  // Access handles
  Handle(TColStd_HArray1OfReal)    hKnots = MResolver.GetOpenCascadeKnots();
  Handle(TColStd_HArray1OfInteger) hMults = MResolver.GetOpenCascadeMults();

  // Access actual knots and multiplicities.
  const TColStd_Array1OfReal&    occtKnots = hKnots->Array1();
  const TColStd_Array1OfInteger& occtMults = hMults->Array1();

  // Build OCCT curve from scratch.
  m_occtCurve = new Geom_BSplineCurve(occtPoles, occtKnots, occtMults, srcDeg);
  m_fMaxError = 0.0;
  m_bIsDone   = true;
}

//-----------------------------------------------------------------------------

void mobius::cascade_BSplineCurve3D::convertToMobius()
{
  const TColgp_Array1OfPnt&      srcPoles = m_occtCurve->Poles();
  const TColStd_Array1OfReal&    srcKnots = m_occtCurve->Knots();
  const TColStd_Array1OfInteger& srcMults = m_occtCurve->Multiplicities();
  const int                      srcDeg   = m_occtCurve->Degree();

  // Poles are transferred as is
  std::vector<xyz> mobiusPoles;
  //
  for ( int i = srcPoles.Lower(); i <= srcPoles.Upper(); ++i )
  {
    xyz P( srcPoles(i).X(), srcPoles(i).Y(), srcPoles(i).Z() );
    mobiusPoles.push_back(P);
  }

  // Fill array of Mobius knots just repeating OCCT knots as many times
  // as multiplicity value dictates.
  std::vector<double> mobiusU;
  //
  for ( int k = srcKnots.Lower(); k <= srcKnots.Upper(); ++k )
  {
    // Get multiplicity.
    const int mult = srcMults(k);

    // Fill knots.
    for ( int m = 0; m < mult; ++m )
      mobiusU.push_back( srcKnots(k) );
  }

  // Build Mobius curve from scratch
  m_mobiusCurve = new bcurve(mobiusPoles, mobiusU, srcDeg);
  m_fMaxError   = 0.0;
  m_bIsDone     = true;
}
