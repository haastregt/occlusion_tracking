cmake_minimum_required(VERSION 3.0.0)
project(cpp_occlusions VERSION 0.1.0)

# Make sure VSCode can find the includes
set(CMAKE_EXPORT_COMPILE_COMMANDS=ON)

###########################
# CGAL and its components #
###########################
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

#############
# Own stuff #
#############

# Src
AUX_SOURCE_DIRECTORY(src SRC_FILES)

# Headers
set(PROJECT_SOURCE_DIR "src")
set(PROJECT_INCLUDE_DIR "include/cpp_occlusions")

# Source files
set(SOURCE_FILES
    ${PROJECT_INCLUDE_DIR}/occluded_volume.h
    ${PROJECT_SOURCE_DIR}/occluded_volume.cpp

    ${PROJECT_INCLUDE_DIR}/occlusion_handler.h
    ${PROJECT_SOURCE_DIR}/occlusion_handler.cpp
)

# Set up such that XCode organizes the files correctly
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCE_FILES})

# Add library
add_library(cpp_occlusions SHARED ${SOURCE_FILES})

# Include directories
target_include_directories(cpp_occlusions PRIVATE include/cpp_occlusions)
if(CGAL_Qt5_FOUND)
  message(STATUS "Found CGAL_Qt5")
  add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)
else()
  message(STATUS "Not Found")
endif()
target_link_libraries(cpp_occlusions PUBLIC CGAL::CGAL CGAL::CGAL_Core CGAL::CGAL_Qt5)


# Install
install(TARGETS cpp_occlusions DESTINATION lib)

# Create base directory
install(DIRECTORY include/cpp_occlusions DESTINATION /usr/include)