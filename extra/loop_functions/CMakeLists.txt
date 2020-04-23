#include_directories(${CMAKE_SOURCE_DIR})

#if(ARGOS_COMPILE_QTOPENGL)
#    add_subdirectory(trajectory_loop_functions)
#endif(ARGOS_COMPILE_QTOPENGL)


add_library(trajectory_loop_functions MODULE 
	map_structure.h
	map_structure.cpp

	robot.h
	robot.cpp
	
	map_elements.h
	map_elements.cpp

	trajectory_loop_functions.h
	trajectory_qtuser_functions.h
	
	trajectory_loop_functions.cpp
	trajectory_qtuser_functions.cpp
)


target_link_libraries(
	trajectory_loop_functions
	argos3core_simulator
	argos3plugin_simulator_entities
	argos3plugin_simulator_footbot
	argos3plugin_simulator_qtopengl

	/usr/include/nlohmann/json.hpp
	# this lib was added, cause Get Sensor, doesnt works!.
	argos3plugin_simulator_genericrobot
${ARGOS_QTOPENGL_LIBRARIES})

