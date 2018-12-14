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
  clouds/geom_PointCloud.h
  clouds/geom_PositionCloud.h
  clouds/geom_SectionCloud.h
  clouds/geom_SectionLine.h
  clouds/geom_VectorField.h
)
set (clouds_CPP_FILES
  clouds/geom_PointCloud.cpp
  clouds/geom_PositionCloud.cpp
  clouds/geom_SectionCloud.cpp
  clouds/geom_SectionLine.cpp
  clouds/geom_VectorField.cpp
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
)
set (common_CPP_FILES
  common/geom_BSurfNk.cpp
)

#------------------------------------------------------------------------------
# Fairing
#------------------------------------------------------------------------------

set (fairing_H_FILES
  fairing/geom_FairBCurve.h
  fairing/geom_FairBCurveAij.h
  fairing/geom_FairBCurveBj.h
  fairing/geom_FairBCurveCoeff.h
  fairing/geom_FairBSurf.h
  fairing/geom_FairBSurfAkl.h
  fairing/geom_FairBSurfBl.h
  fairing/geom_FairBSurfCoeff.h
  fairing/geom_FairingMemBlocks.h
)
set (fairing_CPP_FILES
  fairing/geom_FairBCurve.cpp
  fairing/geom_FairBCurveAij.cpp
  fairing/geom_FairBCurveBj.cpp
  fairing/geom_FairBSurf.cpp
  fairing/geom_FairBSurfAkl.cpp
  fairing/geom_FairBSurfBl.cpp
)

#------------------------------------------------------------------------------
# Primitives
#------------------------------------------------------------------------------

set (primitives_H_FILES
  primitives/geom_BezierOnRailsSurface.h
  primitives/geom_BSplineCurve.h
  primitives/geom_BSplineSurface.h
  primitives/geom_Circle.h
  primitives/geom_CoonsSurface.h
  primitives/geom_Curve.h
  primitives/geom_Geometry.h
  primitives/geom_GordonSurface.h
  primitives/geom_KleinBottle.h
  primitives/geom_KleinIsoCurve.h
  primitives/geom_Line.h
  primitives/geom_Link.h
  primitives/geom_Point.h
  primitives/geom_PolyLine.h
  primitives/geom_SectionPatch.h
  primitives/geom_SphereSurface.h
  primitives/geom_Surface.h
)
set (primitives_CPP_FILES
  primitives/geom_BezierOnRailsSurface.cpp
  primitives/geom_BSplineCurve.cpp
  primitives/geom_BSplineSurface.cpp
  primitives/geom_Circle.cpp
  primitives/geom_CoonsSurface.cpp
  primitives/geom_Curve.cpp
  primitives/geom_Geometry.cpp
  primitives/geom_GordonSurface.cpp
  primitives/geom_KleinBottle.cpp
  primitives/geom_KleinIsoCurve.cpp
  primitives/geom_Line.cpp
  primitives/geom_Link.cpp
  primitives/geom_Point.cpp
  primitives/geom_PolyLine.cpp
  primitives/geom_SphereSurface.cpp
  primitives/geom_Surface.cpp
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

include_directories(${geom_include_dir_loc} ${core_include_dir} ${3RDPARTY_EIGEN_DIR})

if (USE_ADOLC)
  include_directories(${3RDPARTY_adolc_INCLUDE_DIR})
endif()

add_library (mobiusGeom SHARED
  ${H_FILES}               ${CPP_FILES}
  ${classifiers_H_FILES}   ${classifiers_CPP_FILES}
  ${common_H_FILES}        ${common_CPP_FILES}
  ${fairing_H_FILES}       ${fairing_CPP_FILES}
  ${clouds_H_FILES}        ${clouds_CPP_FILES}
  ${interpolation_H_FILES} ${interpolation_CPP_FILES}
  ${primitives_H_FILES}    ${primitives_CPP_FILES})

set (X_COMPILER_BITNESS "x${COMPILER_BITNESS}")

#------------------------------------------------------------------------------
# Dependencies
#------------------------------------------------------------------------------

target_link_libraries(mobiusGeom mobiusCore mobiusBSpl)

#------------------------------------------------------------------------------
# Installation of Mobius as SDK
#------------------------------------------------------------------------------

install (TARGETS mobiusGeom
         RUNTIME DESTINATION bin COMPONENT Runtime
         CONFIGURATIONS Release
         ARCHIVE DESTINATION lib COMPONENT Library
         CONFIGURATIONS Release)

install (TARGETS mobiusGeom
         RUNTIME DESTINATION bini COMPONENT Runtime
         CONFIGURATIONS RelWithDebInfo
         ARCHIVE DESTINATION libi COMPONENT Library
         CONFIGURATIONS RelWithDebInfo)

install (TARGETS mobiusGeom
         RUNTIME DESTINATION bind COMPONENT Runtime
         CONFIGURATIONS Debug
         ARCHIVE DESTINATION libd COMPONENT Library
         CONFIGURATIONS Debug)

install (FILES ${PROJECT_BINARY_DIR}/../../${PLATFORM}${COMPILER_BITNESS}/${COMPILER}/bind/mobiusGeom.pdb DESTINATION bind CONFIGURATIONS Debug)
install (FILES ${PROJECT_BINARY_DIR}/../../${PLATFORM}${COMPILER_BITNESS}/${COMPILER}/bini/mobiusGeom.pdb DESTINATION bini CONFIGURATIONS RelWithDebInfo)

install (FILES ${H_FILES}               DESTINATION include/mobius)
install (FILES ${classifiers_H_FILES}   DESTINATION include/mobius)
install (FILES ${common_H_FILES}        DESTINATION include/mobius)
install (FILES ${fairing_H_FILES}       DESTINATION include/mobius)
install (FILES ${clouds_H_FILES}        DESTINATION include/mobius)
install (FILES ${interpolation_H_FILES} DESTINATION include/mobius)
install (FILES ${primitives_H_FILES}    DESTINATION include/mobius)
