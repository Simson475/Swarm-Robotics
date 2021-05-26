#include_directories(${CMAKE_SOURCE_DIR})

#if(ARGOS_COMPILE_QTOPENGL)
#    add_subdirectory(CTrajectoryLoopFunctions)
#endif(ARGOS_COMPILE_QTOPENGL)

find_package(Qt5Widgets)
find_package(Lua)

include_directories(${LUA_INCLUDE_DIR})
#option(ARGOS_WITH_LUA "ARGOS_WITH_LUA" OFF)
#set(-DARGOS_WITH_LUA "OFF")

#set(ARGOS_WITH_LUA OFF)
unset(ARGOS_WITH_LUA CACHE)

add_library(CTrajectoryLoopFunctions MODULE

        ../models/map/box.cpp
        ../models/map/box.hpp

        ../models/map/point.cpp
        ../models/map/point.hpp

        ../models/map/line.cpp
        ../models/map/line.hpp

        ../models/map/map_structure.hpp
        ../models/map/map_structure.cpp

        ../models/robot/robot.hpp
        ../models/robot/robot.cpp

        ../controllers/parsing/uppaal_model_parsing.hpp
        ../controllers/parsing/uppaal_model_parsing.cpp

        ../argos_wrapper/argos_wrapper.hpp
        ../argos_wrapper/argos_wrapper.cpp

	    CTrajectoryLoopFunctions.h
	    CTrajectoryQTUserFunctions.h

	    CTrajectoryLoopFunctions.cpp
	    CTrajectoryQTUserFunctions.cpp

        ../models/jobs/JobGenerator.hpp
        ../models/jobs/JobGenerator.cpp

        ../models/jobs/PredefinedJobGenerator.hpp
        ../models/jobs/PredefinedJobGenerator.cpp

        ../models/jobs/PredefinedDescreteJobGenerator.hpp
        ../models/jobs/PredefinedDescreteJobGenerator.cpp

        ../models/jobs/Job.hpp
        ../models/jobs/Job.cpp

        ../models/jobs/JobBlueprint.hpp
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