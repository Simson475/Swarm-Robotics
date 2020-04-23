cmake_minimum_required(VERSION 2.8.12)
project(extra)


# Set the path where CMake will find additional scripts
set(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)
# Find the ARGoS package, make sure to save the ARGoS prefix
find_package(PkgConfig)
pkg_check_modules(ARGOS REQUIRED argos3_simulator)
set(ARGOS_PREFIX ${ARGOS_PREFIX} CACHE INTERNAL "")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)

# Check whether all the necessary libs have been installed to compile the
# code that depends on Qt and OpenGL
include(ARGoSCheckQTOpenGL)

# Look for the (optional) galib library
#find_package(GALIB)
#if(GALIB_FOUND)
#  include_directories(${GALIB_INCLUDE_DIRS})
#endif(GALIB_FOUND)


# Find the Lua package

find_package(Lua52 REQUIRED)

# Imgui part
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)



# Find Java
#find_package(Java REQUIRED)

# Find JNI
#find_package(JNI REQUIRED)


# Emit all warning messages
add_definitions(-Wall)

# Set up C++ include directories
# include_directories(${ARGOS_INCLUDE_DIRS} ${JNI_INCLUDE_DIRS} ${LUA_INCLUDE_DIR})

#include_directories(${ARGOS_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS} ${LUA_INCLUDE_DIR})

# Set up C++ link directories

link_directories(${ARGOS_LIBRARY_DIRS})

message( ${CMAKE_CURRENT_SOURCE_DIR} )
message( ${ARGOS_LIBRARY_DIRS} ) 


#add_library(mylib MODULE  test.cpp)
#target_link_libraries(test
#  argos3core_simulator
#  argos3plugin_simulator_footbot
#  argos3plugin_simulator_genericrobot)

add_subdirectory(controllers)

# Descend into the loop_functions directory
add_subdirectory(loop_functions)

#add_subdirectory(imgui)



#add_executable(test test.cpp query_plugins.cpp SHARED ${ARGOS3_MULTIVESTA_WRAPPER_SOURCES})

#add_executable(foo test.cpp query_plugins.cpp)

#target_link_libraries(testa ${ARGOS_LIBRARY_DIRS})

#target_link_libraries(foo argos3core_simulator)
