cmake_minimum_required(VERSION 3.1)
project(occlusion_package VERSION 0.0.1)

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
# Own Package with PyBind11 #
#############################

file (GLOB SOURCE_FILES "src/*.cpp")
file (GLOB HEADER_FILES "include/cpp_occlusions/*.h")

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCE_FILES} ${HEADER_FILES})


find_package(pybind11 REQUIRED)
pybind11_add_module(py_occlusions 
    ${SOURCE_FILES}
    ${HEADER_FILES}
)

target_include_directories(py_occlusions PRIVATE include/cpp_occlusions)

add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)

target_link_libraries(py_occlusions PUBLIC CGAL::CGAL CGAL::CGAL_Core CGAL::CGAL_Qt5)

install(TARGETS py_occlusions 
  COMPONENT python 
  LIBRARY DESTINATION "${PYTHON_LIBRARY_DIR}"
  )