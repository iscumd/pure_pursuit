#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Modify code after this type of comment block only
# Uncomment the following three lines to buildalone
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
project(pure_pursuit)
cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 14)


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Name your Library
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
set(Library_NAME pure_pursuit)

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# VERSION your lib
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
set(Library_VERSION 0.0.1)

# Add cmake directory to cmake module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Let files in the build tree ref files in source tree
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_INCLUDE_CURRENT_DIR_IN_INTERFACE ON)

#CMake module with functions for exporting libs
include(GenerateExportHeader)

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Set your src files
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
set(SRC_FILES
        src/pure_pursuit.cpp)

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Declare your lib
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
add_library(${Library_NAME} SHARED ${SRC_FILES})
add_library(ISC::${Library_NAME} ALIAS ${Library_NAME})

# NOTE: header files in include/ are exported with lib
# private/non-API headers should go in src folder
target_include_directories(${Library_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE
        src
        )

# CMake install code
# This should make lib importable by other projects
# as well as make an install target in the makefile
# that installs the lib on the system
generate_export_header(${Library_NAME})

# VERSION from above
set_property(TARGET ${Library_NAME} PROPERTY VERSION ${Library_VERSION})
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Make sure your library version matches project version
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
set_property(TARGET ${Library_NAME} PROPERTY SOVERSION 0) # Here
set_property(TARGET ${Library_NAME} PROPERTY
        INTERFACE_${Library_NAME}_MAJOR_VERSION 0) # And here
set_property(TARGET ${Library_NAME} APPEND PROPERTY
        COMPATIBLE_INTERFACE_STRING ${Library_NAME}_MAJOR_VERSION
        )

# Standard install structure
install(TARGETS ${Library_NAME} EXPORT ${Library_NAME}Targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
        )

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# API files and Cmake export/import file.
# Add public header files here.
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
install(
        FILES
        include/pure_pursuit.h
        "${CMAKE_CURRENT_BINARY_DIR}/${Library_NAME}_export.h"
        DESTINATION
        include
        COMPONENT
        Devel
)

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# CMake modules to write package.
# Can modify Namespace for now, may settle on one.
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/${Library_NAME}/${Library_NAME}ConfigVersion.cmake"
        VERSION ${Library_VERSION}
        COMPATIBILITY AnyNewerVersion
)
export(EXPORT ${Library_NAME}Targets
        FILE "${CMAKE_CURRENT_BINARY_DIR}/${Library_NAME}/${Library_NAME}Targets.cmake"
        NAMESPACE isc:: # NAMESPACE FOR LIB
        )
configure_file(cmake/${Library_NAME}Config.cmake
        "${CMAKE_CURRENT_BINARY_DIR}/${Library_NAME}/${Library_NAME}Config.cmake"
        COPYONLY
        )
set(ConfigPackageLocation lib/cmake/${Library_NAME})
install(EXPORT ${Library_NAME}Targets
        FILE
        ${Library_NAME}Targets.cmake
        NAMESPACE
        isc::
        DESTINATION
        ${ConfigPackageLocation}
        )
install(
        FILES
        cmake/${Library_NAME}Config.cmake
        "${CMAKE_CURRENT_BINARY_DIR}/${Library_NAME}/${Library_NAME}ConfigVersion.cmake"
        DESTINATION
        ${ConfigPackageLocation}
        COMPONENT
        Devel
)


# Test configuration
# Note: Distribute header with each module to facilitate individual builds if required
set(CATCH_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/catch)
set("TEST_LIB_${Library_NAME}" "Catch_${Library_NAME}")
add_library("TEST_LIB_${Library_NAME}" INTERFACE)
target_include_directories("TEST_LIB_${Library_NAME}" INTERFACE ${CATCH_INCLUDE_DIR})

file(GLOB_RECURSE TEST_FILES ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp)
set(Test_Name "TEST_${Library_NAME}.test")
add_executable(${Test_Name} ${TEST_FILES})
target_link_libraries(${Test_Name} PRIVATE "TEST_LIB_${Library_NAME}" ${Library_NAME})