project (mobiusGeom)

#------------------------------------------------------------------------------
# Common
#------------------------------------------------------------------------------

set (H_FILES
  geom.h
  geom_excBCurveCtor.h
  geom_excBSurfaceCtor.h
  geom_JSON.h
)
set (CPP_FILES
  geom_JSON.cpp
)

#------------------------------------------------------------------------------
# Approximation
#------------------------------------------------------------------------------

set (approximation_H_FILES
  approximation/geom_ApproxBSurf.h
  approximation/geom_ApproxBSurfBi.h
  approximation/geom_ApproxBSurfCoeff.h
  approximation/geom_ApproxBSurfMij.h
)
set (approximation_CPP_FILES
  approximation/geom_ApproxBSurf.cpp
  approximation/geom_ApproxBSurfBi.cpp
  approximation/geom_ApproxBSurfMij.cpp
)

#------------------------------------------------------------------------------
# Classifiers
#------------------------------------------------------------------------------

set (classifiers_H_FILES
  classifiers/geom_PointOnLine.h
)
set (classifiers_CPP_FILES
  classifiers/geom_PointOnLine.cpp
)

#------------------------------------------------------------------------------
# Point clouds
#------------------------------------------------------------------------------

set (clouds_H_FILES
  clouds/geom_BuildAveragePlane.h
  clouds/geom_PointCloud.h
  clouds/geom_PositionCloud.h
  clouds/geom_SectionCloud.h
  clouds/geom_SectionLine.h
  clouds/geom_VectorField.h
)
set (clouds_CPP_FILES
  clouds/geom_BuildAveragePlane.cpp
  clouds/geom_PointCloud.cpp
  clouds/geom_PositionCloud.cpp
  clouds/geom_SectionCloud.cpp
  clouds/geom_SectionLine.cpp
  clouds/geom_VectorField.cpp
)

#------------------------------------------------------------------------------
# Interoperability
#------------------------------------------------------------------------------

set (interop_H_FILES
  interop/geom_ReadAstra.h
)
set (interop_CPP_FILES
  interop/geom_ReadAstra.cpp
)

#------------------------------------------------------------------------------
# Interpolation
#------------------------------------------------------------------------------

set (interpolation_H_FILES
  interpolation/geom_InterpolateCurve.h
  interpolation/geom_InterpolateMultiCurve.h
  interpolation/geom_InterpolateSurface.h
  interpolation/geom_SkinSurface.h
)
set (interpolation_CPP_FILES
  interpolation/geom_InterpolateCurve.cpp
  interpolation/geom_InterpolateMultiCurve.cpp
  interpolation/geom_InterpolateSurface.cpp
  interpolation/geom_SkinSurface.cpp
)

#------------------------------------------------------------------------------
# Common
#------------------------------------------------------------------------------

set (common_H_FILES
  common/geom_BSurfNk.h
  common/geom_MakeBicubicBSurf.h
  common/geom_OptimizeBSurfBase.h
  common/geom_UnifyBCurves.h
)
set (common_CPP_FILES
  common/geom_BSurfNk.cpp
  common/geom_MakeBicubicBSurf.cpp
  common/geom_UnifyBCurves.cpp
)

#------------------------------------------------------------------------------
# Fairing
#------------------------------------------------------------------------------

set (fairing_H_FILES
  fairing/geom_FairBCurve.h
  fairing/geom_FairBCurveAij.h
  fairing/geom_FairBCurveBi.h
  fairing/geom_FairBCurveCoeff.h
  fairing/geom_FairBSurf.h
  fairing/geom_FairBSurfAkl.h
  fairing/geom_FairBSurfBk.h
  fairing/geom_FairBSurfCoeff.h
  fairing/geom_FairingMemBlocks.h
)
set (fairing_CPP_FILES
  fairing/geom_FairBCurve.cpp
  fairing/geom_FairBCurveAij.cpp
  fairing/geom_FairBCurveBi.cpp
  fairing/geom_FairBSurf.cpp
  fairing/geom_FairBSurfAkl.cpp
  fairing/geom_FairBSurfBk.cpp
)

#------------------------------------------------------------------------------
# Primitives
#------------------------------------------------------------------------------

set (primitives_H_FILES
  primitives/geom_BezierOnRailsSurface.h
  primitives/geom_BSplineCurve.h
  primitives/geom_BSplineSurface.h
  primitives/geom_Circle.h
  primitives/geom_CoonsSurfaceCubic.h
  primitives/geom_CoonsSurfaceLinear.h
  primitives/geom_Curve.h
  primitives/geom_Geometry.h
  primitives/geom_GordonSurface.h
  primitives/geom_KleinBottle.h
  primitives/geom_KleinIsoCurve.h
  primitives/geom_Line.h
  primitives/geom_Link.h
  primitives/geom_PlaneSurface.h
  primitives/geom_Point.h
  primitives/geom_PolyLine.h
  primitives/geom_SectionPatch.h
  primitives/geom_SphereSurface.h
  primitives/geom_Surface.h
  primitives/geom_SurfaceOfRevolution.h
)
set (primitives_CPP_FILES
  primitives/geom_BezierOnRailsSurface.cpp
  primitives/geom_BSplineCurve.cpp
  primitives/geom_BSplineSurface.cpp
  primitives/geom_Circle.cpp
  primitives/geom_CoonsSurfaceCubic.cpp
  primitives/geom_CoonsSurfaceLinear.cpp
  primitives/geom_Curve.cpp
  primitives/geom_Geometry.cpp
  primitives/geom_GordonSurface.cpp
  primitives/geom_KleinBottle.cpp
  primitives/geom_KleinIsoCurve.cpp
  primitives/geom_Line.cpp
  primitives/geom_Link.cpp
  primitives/geom_PlaneSurface.cpp
  primitives/geom_Point.cpp
  primitives/geom_PolyLine.cpp
  primitives/geom_SphereSurface.cpp
  primitives/geom_Surface.cpp
  primitives/geom_SurfaceOfRevolution.cpp
)

#------------------------------------------------------------------------------
# OCC tools
#------------------------------------------------------------------------------

set (occ_H_FILES
  occ/Bnd_Box2d.hxx
  occ/ElSLib.hxx
  occ/gp.hxx
  occ/gp_Ax1.hxx
  occ/gp_Ax2.hxx
  occ/gp_Ax2d.hxx
  occ/gp_Ax3.hxx
  occ/gp_Circ.hxx
  occ/gp_Cylinder.hxx
  occ/gp_Cone.hxx
  occ/gp_Dir.hxx
  occ/gp_Dir2d.hxx
  occ/gp_GTrsf.hxx
  occ/gp_GTrsf2d.hxx
  occ/gp_Lin.hxx
  occ/gp_Lin2d.hxx
  occ/gp_Mat.hxx
  occ/gp_Mat2d.hxx
  occ/gp_Pnt.hxx
  occ/gp_Pln.hxx
  occ/gp_Pnt2d.hxx
  occ/gp_Quaternion.hxx
  occ/gp_QuaternionNLerp.hxx
  occ/gp_QuaternionSLerp.hxx
  occ/gp_Sphere.hxx
  occ/gp_Torus.hxx
  occ/gp_Trsf.hxx
  occ/gp_Trsf2d.hxx
  occ/gp_TrsfForm.hxx
  occ/gp_Vec.hxx
  occ/gp_Vec2d.hxx
  occ/gp_XY.hxx
  occ/gp_XYZ.hxx
  occ/Intf.hxx
  occ/Intf_Array1OfLin.hxx
  occ/Intf_Interference.hxx
  occ/Intf_InterferencePolygon2d.hxx
  occ/Intf_PIType.hxx
  occ/Intf_Polygon2d.hxx
  occ/Intf_SectionLine.hxx
  occ/Intf_SectionPoint.hxx
  occ/Intf_TangentZone.hxx
  occ/Intf_SeqOfSectionLine.hxx
  occ/Intf_SeqOfSectionPoint.hxx
  occ/Intf_SeqOfTangentZone.hxx
  occ/NCollection_BaseAllocator.hxx
  occ/NCollection_BaseList.hxx
  occ/NCollection_BaseMap.hxx
  occ/NCollection_BaseSequence.hxx
  occ/NCollection_DataMap.hxx
  occ/NCollection_DefaultHasher.hxx
  occ/NCollection_DefineAlloc.hxx
  occ/NCollection_IncAllocator.hxx
  occ/NCollection_List.hxx
  occ/NCollection_ListNode.hxx
  occ/NCollection_Map.hxx
  occ/NCollection_Sequence.hxx
  occ/NCollection_Shared.hxx
  occ/NCollection_StlIterator.hxx
  occ/NCollection_TListIterator.hxx
  occ/NCollection_TListNode.hxx
  occ/NCollection_TypeDef.hxx
  occ/Standard.hxx
  occ/Standard_Address.hxx
  occ/Standard_Assert.hxx
  occ/Standard_Atomic.hxx
  occ/Standard_Boolean.hxx
  occ/Standard_Byte.hxx
  occ/Standard_Character.hxx
  occ/Standard_CLocaleSentry.hxx
  occ/Standard_CString.hxx
  occ/Standard_DefineAlloc.hxx
  occ/Standard_DefineException.hxx
  occ/Standard_DomainError.hxx
  occ/Standard_ErrorHandler.hxx
  occ/Standard_Failure.hxx
  occ/Standard_Handle.hxx
  occ/Standard_HandlerStatus.hxx
  occ/Standard_Integer.hxx
  occ/Standard_JmpBuf.hxx
  occ/Standard_Macro.hxx
  occ/Standard_math.hxx
  occ/Standard_MMgrOpt.hxx
  occ/Standard_MMgrRaw.hxx
  occ/Standard_MMgrRoot.hxx
  occ/Standard_MMgrTBBalloc.hxx
  occ/Standard_Mutex.hxx
  occ/Standard_NoMoreObject.hxx
  occ/Standard_NoSuchObject.hxx
  occ/Standard_NotImplemented.hxx
  occ/Standard_NullObject.hxx
  occ/Standard_NullValue.hxx
  occ/Standard_NumericError.hxx
  occ/Standard_OStream.hxx
  occ/Standard_OutOfMemory.hxx
  occ/Standard_OutOfRange.hxx
  occ/Standard_PErrorHandler.hxx
  occ/Standard_PrimitiveTypes.hxx
  occ/Standard_ProgramError.hxx
  occ/Standard_RangeError.hxx
  occ/Standard_Real.hxx
  occ/Standard_ShortReal.hxx
  occ/Standard_Size.hxx
  occ/Standard_Std.hxx
  occ/Standard_SStream.hxx
  occ/Standard_Stream.hxx
  occ/Standard_ThreadId.hxx
  occ/Standard_Transient.hxx
  occ/Standard_Type.hxx
  occ/Standard_TypeDef.hxx
  occ/Standard_Values.hxx
  occ/occMathDefs.hxx
)
set (occ_CPP_FILES
  occ/Bnd_Box2d.cxx
  occ/ElSLib.cxx
  occ/gp.cxx
  occ/gp_Ax1.cxx
  occ/gp_Ax2.cxx
  occ/gp_Ax2d.cxx
  occ/gp_Ax3.cxx
  occ/gp_Circ.cxx
  occ/gp_Circ.lxx
  occ/gp_Cylinder.cxx
  occ/gp_Cone.cxx
  occ/gp_Dir.cxx
  occ/gp_Dir2d.cxx
  occ/gp_GTrsf.cxx
  occ/gp_GTrsf2d.cxx
  occ/gp_Lin.cxx
  occ/gp_Lin2d.cxx
  occ/gp_Mat.cxx
  occ/gp_Mat2d.cxx
  occ/gp_Pln.cxx
  occ/gp_Pnt.cxx
  occ/gp_Pnt2d.cxx
  occ/gp_Quaternion.cxx
  occ/gp_Sphere.cxx
  occ/gp_Torus.cxx
  occ/gp_Trsf.cxx
  occ/gp_Trsf2d.cxx
  occ/gp_Vec.cxx
  occ/gp_Vec2d.cxx
  occ/gp_XY.cxx
  occ/gp_XYZ.cxx
  occ/Intf.cxx
  occ/Intf_Interference.cxx
  occ/Intf_InterferencePolygon2d.cxx
  occ/Intf_Polygon2d.cxx
  occ/Intf_SectionLine.cxx
  occ/Intf_SectionPoint.cxx
  occ/Intf_TangentZone.cxx
  occ/NCollection_BaseAllocator.cxx
  occ/NCollection_BaseList.cxx
  occ/NCollection_BaseMap.cxx
  occ/NCollection_BaseSequence.cxx
  occ/NCollection_IncAllocator.cxx
  occ/Standard.cxx
  occ/Standard_CLocaleSentry.cxx
  occ/Standard_CString.cxx
  occ/Standard_ErrorHandler.cxx
  occ/Standard_Failure.cxx
  occ/Standard_OutOfMemory.cxx
  occ/Standard_math.cxx
  occ/Standard_Mutex.cxx
  occ/Standard_MMgrOpt.cxx
  occ/Standard_MMgrRaw.cxx
  occ/Standard_MMgrRoot.cxx
  occ/Standard_MMgrTBBalloc.cxx
  occ/Standard_Real.cxx
  occ/Standard_ShortReal.cxx
  occ/Standard_StackTrace.cxx
  occ/Standard_Strtod.cxx
  occ/Standard_Transient.cxx
  occ/Standard_Type.cxx
)

#------------------------------------------------------------------------------

list (APPEND MOBIUS_MODULES geom)
set (MOBIUS_MODULES ${MOBIUS_MODULES} PARENT_SCOPE)

#------------------------------------------------------------------------------

# Create include variable
set (geom_include_dir_loc "${CMAKE_BINARY_DIR}/inc")
#
set (geom_include_dir ${geom_include_dir_loc} PARENT_SCOPE)

foreach (FILE ${H_FILES})
  source_group ("Header Files" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${CPP_FILES})
  source_group ("Source Files" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${approximation_H_FILES})
  source_group ("Header Files\\Approximation" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${approximation_CPP_FILES})
  source_group ("Source Files\\Approximation" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${classifiers_H_FILES})
  source_group ("Header Files\\Classifiers" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${classifiers_CPP_FILES})
  source_group ("Source Files\\Classifiers" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${common_H_FILES})
  source_group ("Header Files\\Common" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${common_CPP_FILES})
  source_group ("Source Files\\Common" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${fairing_H_FILES})
  source_group ("Header Files\\Fairing" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${fairing_CPP_FILES})
  source_group ("Source Files\\Fairing" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${clouds_H_FILES})
  source_group ("Header Files\\Points" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${clouds_CPP_FILES})
  source_group ("Source Files\\Points" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${interop_H_FILES})
  source_group ("Header Files\\Interoperability" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${interop_CPP_FILES})
  source_group ("Source Files\\Interoperability" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${interpolation_H_FILES})
  source_group ("Header Files\\Interpolation" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${interpolation_CPP_FILES})
  source_group ("Source Files\\Interpolation" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${primitives_H_FILES})
  source_group ("Header Files\\Primitives" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${primitives_CPP_FILES})
  source_group ("Source Files\\Primitives" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${occ_H_FILES})
  source_group ("Header Files\\OpenCascade" FILES "${FILE}")
endforeach (FILE)

foreach (FILE ${occ_CPP_FILES})
  source_group ("Source Files\\OpenCascade" FILES "${FILE}")
endforeach (FILE)

include_directories(${geom_include_dir_loc} ${core_include_dir} ${3RDPARTY_EIGEN_DIR})

if (USE_ADOLC)
  include_directories(${3RDPARTY_adolc_INCLUDE_DIR})
endif()

add_library (mobiusGeom SHARED
  ${H_FILES}               ${CPP_FILES}
  ${approximation_H_FILES} ${approximation_CPP_FILES}
  ${classifiers_H_FILES}   ${classifiers_CPP_FILES}
  ${common_H_FILES}        ${common_CPP_FILES}
  ${fairing_H_FILES}       ${fairing_CPP_FILES}
  ${clouds_H_FILES}        ${clouds_CPP_FILES}
  ${interop_H_FILES}       ${interop_CPP_FILES}
  ${interpolation_H_FILES} ${interpolation_CPP_FILES}
  ${primitives_H_FILES}    ${primitives_CPP_FILES}
  ${occ_H_FILES}           ${occ_CPP_FILES}
)

set (X_COMPILER_BITNESS "x${COMPILER_BITNESS}")

#------------------------------------------------------------------------------
# Dependencies
#------------------------------------------------------------------------------

target_link_libraries(mobiusGeom mobiusCore mobiusBSpl)

#------------------------------------------------------------------------------
# Installation of Mobius as SDK
#------------------------------------------------------------------------------

install (TARGETS mobiusGeom
         CONFIGURATIONS Release
         RUNTIME DESTINATION bin COMPONENT Runtime
         ARCHIVE DESTINATION lib COMPONENT Library
         LIBRARY DESTINATION lib COMPONENT Library)

install (TARGETS mobiusGeom
         CONFIGURATIONS RelWithDebInfo
         RUNTIME DESTINATION bini COMPONENT Runtime
         ARCHIVE DESTINATION libi COMPONENT Library
         LIBRARY DESTINATION libi COMPONENT Library)

install (TARGETS mobiusGeom
         CONFIGURATIONS Debug
         RUNTIME DESTINATION bind COMPONENT Runtime
         ARCHIVE DESTINATION libd COMPONENT Library
         LIBRARY DESTINATION libd COMPONENT Library)

if (MSVC)
  install (FILES ${PROJECT_BINARY_DIR}/../../${PLATFORM}${COMPILER_BITNESS}/${COMPILER}/bind/mobiusGeom.pdb DESTINATION bind CONFIGURATIONS Debug)
  install (FILES ${PROJECT_BINARY_DIR}/../../${PLATFORM}${COMPILER_BITNESS}/${COMPILER}/bini/mobiusGeom.pdb DESTINATION bini CONFIGURATIONS RelWithDebInfo)
endif()

install (FILES ${H_FILES}               DESTINATION include/mobius)
install (FILES ${approximation_H_FILES} DESTINATION include/mobius)
install (FILES ${classifiers_H_FILES}   DESTINATION include/mobius)
install (FILES ${common_H_FILES}        DESTINATION include/mobius)
install (FILES ${fairing_H_FILES}       DESTINATION include/mobius)
install (FILES ${clouds_H_FILES}        DESTINATION include/mobius)
install (FILES ${interop_H_FILES}       DESTINATION include/mobius)
install (FILES ${interpolation_H_FILES} DESTINATION include/mobius)
install (FILES ${primitives_H_FILES}    DESTINATION include/mobius)
