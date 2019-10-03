#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

find_package(tinyxml2 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(SOURCES urdf.cpp yaml.cpp)
set(HEADERS RBDyn/parsers/common.h RBDyn/parsers/urdf.h RBDyn/parsers/yaml.h)

add_library(RBDynParsers SHARED ${SOURCES} ${HEADERS})
target_include_directories(RBDynParsers PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/../include> $<INSTALL_INTERFACE:include>)
target_link_libraries(RBDynParsers PUBLIC RBDyn tinyxml2::tinyxml2 yaml-cpp)
set_target_properties(RBDynParsers PROPERTIES SOVERSION 1 VERSION 1.0.0)
set_target_properties(RBDynParsers PROPERTIES CXX_STANDARD 11)

install(
    TARGETS RBDynParsers
    EXPORT "${TARGETS_EXPORT_NAME}"
    LIBRARY DESTINATION "lib"
    ARCHIVE DESTINATION "lib"
    RUNTIME DESTINATION "bin"
)
install(FILES ${HEADERS} DESTINATION ${INCLUDE_INSTALL_DESTINATION})
