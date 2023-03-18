cmake_minimum_required(VERSION 3.1)
project(cpp_occlusions VERSION 0.0.1)

# Make sure VSCode can find the includes
set(CMAKE_EXPORT_COMPILE_COMMANDS=ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

#############################
# CGAL and its dependencies #
#############################
find_package( CGAL REQUIRED COMPONENTS Core Qt5 )

if ( NOT CGAL_FOUND )
  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  
endif()


# Boost and its components
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )
  message(STATUS "This project requires the Boost library, and will not be compiled.")
  return()  
endif()

#############################
#  Link c++ code with CGAL  #
#############################

# Src
#AUX_SOURCE_DIRECTORY(src SRC_FILES)

file (GLOB SOURCE_FILES "cpp_occlusions/src/*.cpp")
file (GLOB HEADER_FILES "cpp_occlusions/include/cpp_occlusions/*.h")
file (GLOB PYTHON_FILES "python/cpp_bindings.cpp")

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCE_FILES} ${HEADER_FILES} ${PYTHON_FILES} )


# Add library
#add_library(occlusions SHARED ${SOURCE_FILES} ${HEADER_FILES})



add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)
# Not sure if this is needed
#include_directories("${CMAKE_SOURCE_DIR}/cpp_occlusions/include/cpp_occlusions")
#include_directories("${CMAKE_SOURCE_DIR}/python")

#############################
# Own Package with PyBind11 #
#############################
find_package(pybind11 REQUIRED)
pybind11_add_module(cpp_occlusions 
    ${SOURCE_FILES}
    ${HEADER_FILES}
    ${PYTHON_FILES}
)

# Include directories
target_include_directories(cpp_occlusions PRIVATE cpp_occlusions/include/cpp_occlusions)

add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)
target_link_libraries(cpp_occlusions PUBLIC CGAL::CGAL CGAL::CGAL_Core CGAL::CGAL_Qt5)

install(TARGETS cpp_occlusions 
  COMPONENT python 
  LIBRARY DESTINATION "${PYTHON_LIBRARY_DIR}"
  )