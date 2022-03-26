include_directories(.)
include_directories(../Pathfinding/)
include_directories(../Pathfinding/ConstraintTree/)
include_directories(../Pathfinding/Interfaces/)
include_directories(../models/map/)

# Find the QtWidgets library
find_package(Qt5Widgets) #@todo: Is this necessary anymore?
find_package(Lua)

include_directories(${LUA_INCLUDE_DIR})

#option(ARGOS_WITH_LUA "ARGOS_WITH_LUA" OFF)
#set(ARGOS_WITH_LUA "OFF")

#set(ARGOS_WITH_LUA OFF)
#add_definitions(-DARGOS_WITH_LUA=0)
unset(ARGOS_WITH_LUA CACHE)

set(PathfindingFiles
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/ConstraintTree/Conflict.hpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/ConstraintTree/Conflict.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/ConstraintTree/ConstraintTree.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/ConstraintTree/ConstraintTree.hpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/Action.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/Action.hpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/ActionPathAux.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/ActionPathAux.hpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/ExperimentData.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/ExperimentData.hpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/Agent.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/Agent.hpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/ConstraintTree/Constraint.hpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/HighLevelCBS.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/HighLevelCBS.hpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/LowLevelCBS.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/LowLevelCBS.hpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/_path.hpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/_path.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/Solution.hpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/Solution.cpp

        ${CMAKE_CURRENT_LIST_DIR}/TestController.hpp
        ${CMAKE_CURRENT_LIST_DIR}/TestController.cpp

        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/Interfaces/Edge.hpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/Interfaces/Edge.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/Interfaces/Graph.hpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/Interfaces/Graph.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/Interfaces/Vertex.hpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/Interfaces/Vertex.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/AgentInfo.hpp
        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/AgentInfo.cpp

        ${CMAKE_CURRENT_LIST_DIR}/../models/map/map_structure.cpp
        ${CMAKE_CURRENT_LIST_DIR}/../models/map/map_structure.hpp

        ${CMAKE_CURRENT_LIST_DIR}/../Pathfinding/Debugging.hpp
)

# Add a library target called BotImplementations from the source files listed
# MODULE makes it a library/plugin that is not linked into other targets but may be loaded dynamically at runtime
add_library(BotImplementations MODULE
        ./exceptions/StrategySynthesisError.hpp

        ./RobotInterface/RobotInterface.hpp
        ./RobotInterface/RobotInterface.cpp


        SingleThreadUppaalBot.hpp
        SingleThreadUppaalBot.cpp

        SingleThreadBotGreedy.cpp
        SingleThreadBotGreedy.hpp

        DebugBot.cpp
        DebugBot.hpp

        GreedyStationUppaalWaypoint.cpp
        GreedyStationUppaalWaypoint.hpp

        UppaalStationGreedyWaypoint.cpp
        UppaalStationGreedyWaypoint.hpp
        
        #SingleThreadBotCBS.hpp
        #SingleThreadBotCBS.cpp

        #GreedyStationCBSWaypoint.cpp
        #GreedyStationCBSWaypoint.hpp

        ${PathfindingFiles}
        )

# Ensure that argos-ext is build before BotImplementations
add_dependencies(BotImplementations argos-ext)

# Link the argos3 directory to the BotImplementations
# argos3 installation puts it in its own folder.
target_link_directories(BotImplementations PRIVATE ${EXTERNAL_INSTALL_LOCATION}/lib/argos3)

# Link the libraries to BotImplementations
target_link_libraries(BotImplementations
        Qt5::Widgets# TODO earlier in this file there is a todo to possible remove the Qt5, which would require this to be removed too?
        argos3core_simulator
        argos3plugin_simulator_entities
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot
        argos3plugin_simulator_qtopengl
        )

# Copy the libBotImplementations.so to ../external/bin/libBotImplementations.so
add_custom_command(TARGET BotImplementations POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        ./libBotImplementations.so
        ../external/bin/libBotImplementations.so
        )

# Copy the ../../controllers/parsing/planning_blueprint.xml to <CMAKE_BINARY_DIR>/external/bin/planning_blueprint.xml
add_custom_command(TARGET BotImplementations POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        ../../controllers/parsing/planning_blueprint.xml
        ${CMAKE_BINARY_DIR}/external/bin/planning_blueprint.xml
        )