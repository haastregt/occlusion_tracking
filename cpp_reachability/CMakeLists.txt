cmake_minimum_required(VERSION 3.0.0)
project(reachability VERSION 0.1.0)

# Make sure VSCode can find the includes
set(-DCMAKE_EXPORT_COMPILE_COMMANDS=ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


###########################
# CGAL and its components #
###########################
find_package( CGAL QUIET COMPONENTS  )

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

#############
# Own stuff #
#############

# Include dir
include_directories(/usr/local/include)

# Src
AUX_SOURCE_DIRECTORY(src SRC_FILES)

# Headers
set(PROJECT_SOURCE_DIR "src")
set(PROJECT_INCLUDE_DIR "include/reachability_analysis")

# Source files
set(SOURCE_FILES
    ${PROJECT_INCLUDE_DIR}/reachability_core.h
    ${PROJECT_SOURCE_DIR}/reachability_core.cpp
)

# Set up such that XCode organizes the files correctly
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCE_FILES})

# Add library
add_library(reachability SHARED ${SOURCE_FILES})

# Include directories
target_include_directories(reachability PRIVATE include/reachability_analysis)
target_link_libraries(reachability CGAL::CGAL)


# Install
install(TARGETS reachability DESTINATION lib)

# Install the headers
install(FILES include/reachability DESTINATION include)

# Create base directory
install(DIRECTORY include/reachability_analysis DESTINATION include)