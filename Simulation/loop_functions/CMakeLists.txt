#include_directories(${CMAKE_SOURCE_DIR})

#if(ARGOS_COMPILE_QTOPENGL)
#    add_subdirectory(CTrajectoryLoopFunctions)
#endif(ARGOS_COMPILE_QTOPENGL)

find_package(Qt5Widgets)
find_package(Lua)

include_directories(${LUA_INCLUDE_DIR})

add_library(CTrajectoryLoopFunctions MODULE

../models/box.cpp
../models/box.hpp

../models/point.cpp
../models/point.hpp

../models/line.cpp
../models/line.hpp

../models/map_structure.hpp
../models/map_structure.cpp

../models/robot.hpp
../models/robot.cpp

	CTrajectoryLoopFunctions.h
	CTrajectoryQTUserFunctions.h

	CTrajectoryLoopFunctions.cpp
	CTrajectoryQTUserFunctions.cpp
)


#Because argos3 installation puts in its own folder.
target_link_directories(CTrajectoryLoopFunctions PRIVATE ${EXTERNAL_INSTALL_LOCATION}/lib/argos3)

target_link_libraries(CTrajectoryLoopFunctions
	Qt5::Widgets
	argos3core_simulator
	argos3plugin_simulator_entities
	argos3plugin_simulator_footbot
	argos3plugin_simulator_qtopengl
	# this lib was added, cause Get Sensor, doesnt works!.
	argos3plugin_simulator_genericrobot
${ARGOS_QTOPENGL_LIBRARIES})

add_dependencies(CTrajectoryLoopFunctions json-ext argos-ext)

add_custom_command(TARGET CTrajectoryLoopFunctions POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        ./libCTrajectoryLoopFunctions.so
        ../external/bin/libCTrajectoryLoopFunctions.so
        )