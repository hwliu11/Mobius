project(docalgo)

set (DOX_FILES
  000_!_intro.md
  000_01_getting-started.md
  001_!_architecture.md
  001_01_memory-management.md
  002_!_primitives.md
  002_01_bsurf.md
  003_!_core-algos.md
  003_01_bspl.md
  004_!_operators.md
  004_01_surface-fairing.md
  999_references.md
)

foreach (doxfile ${DOX_FILES})
  set (FULLPATH_DOX_FILES ${FULLPATH_DOX_FILES} "${docalgo_SOURCE_DIR}/pages/${doxfile}")
  source_group ("Documents" FILES "pages/${doxfile}")
endforeach()

set (DOXYGEN_ALGO_DIR "${docalgo_BINARY_DIR}/ALGO")
set (DOXYGEN_ALGO_GENERATED "${DOXYGEN_ALGO_DIR}/generated")
set (DOXYGEN_ALGO_INPUT "${docalgo_SOURCE_DIR}/pages")
set (DOXYGEN_ALGO_IMGS "${docalgo_SOURCE_DIR}/../imgs/algo")

configure_file(${docalgo_SOURCE_DIR}/ALGO_doxyspec
               ${DOXYGEN_ALGO_DIR}/ALGO_doxyspec @ONLY)

configure_file(${DOXYGEN_STATIC}/index_ref.html
               ${DOXYGEN_ALGO_DIR}/ALGO.html COPYONLY)

add_custom_command(OUTPUT ${DOXYGEN_ALGO_GENERATED}/index.html
                   COMMAND ${CMAKE_COMMAND} -E echo_append "Building ALGO documentation..."
                   COMMAND ${DOXYGEN_EXE} ${DOXYGEN_ALGO_DIR}/ALGO_doxyspec
                   COMMAND ${CMAKE_COMMAND} -E echo "Done."
                   DEPENDS ${FULLPATH_DOX_FILES})

add_custom_target(docalgo ALL DEPENDS ${DOXYGEN_ALGO_GENERATED}/index.html SOURCES ${FULLPATH_DOX_FILES})

install (DIRECTORY ${docalgo_BINARY_DIR}/ALGO/generated DESTINATION doc/ALGO)
install (FILES ${docalgo_BINARY_DIR}/ALGO/ALGO.html DESTINATION doc/ALGO)
