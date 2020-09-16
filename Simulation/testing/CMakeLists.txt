cmake_minimum_required(VERSION 2.8.12)
project(testing)
# Set the path where CMake will find additional scripts
set(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)
# Find the ARGoS package, make sure to save the ARGoS prefix
find_package(PkgConfig)
pkg_check_modules(ARGOS REQUIRED argos3_simulator)
set(ARGOS_PREFIX ${ARGOS_PREFIX} CACHE INTERNAL "")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)
include(ARGoSCheckQTOpenGL)
find_package(Lua52 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
add_definitions(-Wall)
link_libraries(stdc++fs)
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
#add_library(applib foo.cpp)

#add_library(looper MODULE ../loop_functions/map_elements.h ../loop_functions/map_elements.cpp )



#add_executable(out ALIAS ../../build/loop_functions/libCTrajectoryLoopFunctions.so )
add_executable(testMain testMain.cpp )
#set_property(TARGET foo PROPERTY IMPORTED_LOCATION "../../build/loop_functions/libCTrajectoryLoopFunctions.so")

target_link_libraries(
	testMain
	argos3core_simulator
	argos3plugin_simulator_entities
	argos3plugin_simulator_footbot
	argos3plugin_simulator_qtopengl
	/home/widok/Desktop/Swarmrobotic/swarmrobotics/extra/build/loop_functions/libCTrajectoryLoopFunctions.so
	/home/widok/Desktop/Swarmrobotic/swarmrobotics/extra/build/controllers/libCFootBotDiffusion.so
	/usr/include/nlohmann/json.hpp
	# this lib was added, cause Get Sensor, doesnt works!.
	argos3plugin_simulator_genericrobot
${ARGOS_QTOPENGL_LIBRARIES})
