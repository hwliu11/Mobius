project(docapi)

set (DOXYGEN_API_DIR "${docapi_BINARY_DIR}/API")
set (DOXYGEN_API_GENERATED "${DOXYGEN_API_DIR}/generated")
set (DOXYGEN_API_TAG "${DOXYGEN_API_GENERATED}/API.tag")
set (DOXYGEN_API_SRC "${CMAKE_SOURCE_DIR}/src")
set (DOXYGEN_API_TAG_LOC "${DOXYGEN_API_TAG}=/${DOXYGEN_API_GENERATED}" PARENT_SCOPE)

configure_file(${docapi_SOURCE_DIR}/API_doxyspec
               ${DOXYGEN_API_DIR}/API_doxyspec @ONLY)

configure_file(${DOXYGEN_STATIC}/index_ref.html
               ${DOXYGEN_API_DIR}/API.html COPYONLY)

add_custom_command(OUTPUT ${DOXYGEN_API_GENERATED}/index.html
                   COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
                   COMMAND ${DOXYGEN_EXE} ${DOXYGEN_API_DIR}/API_doxyspec
                   COMMAND ${CMAKE_COMMAND} -E echo "Done."
                   DEPENDS ${dep_sources})

add_custom_target(docapi ALL DEPENDS ${DOXYGEN_API_GENERATED}/index.html)

install (DIRECTORY ${docapi_BINARY_DIR}/API/generated DESTINATION doc/API)
install (FILES ${docapi_BINARY_DIR}/API/API.html DESTINATION doc/API)
