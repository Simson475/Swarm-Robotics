include_directories(.)
include_directories(../Pathfinding/)
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

set(PathfindingDir ../Pathfinding)
set(ControllersDir .)
set(MapDir ../models/map)
set(PathfindingFiles
        ${PathfindingDir}/Conflict.hpp
        ${PathfindingDir}/Conflict.cpp
        ${PathfindingDir}/ConstraintTree.hpp
        ${PathfindingDir}/ConstraintTree.cpp
        ${PathfindingDir}/Action.hpp
        ${PathfindingDir}/Action.cpp
        ${PathfindingDir}/ActionPathAux.hpp
        ${PathfindingDir}/ActionPathAux.cpp
        ${PathfindingDir}/BackwardsActionPathAux.hpp
        ${PathfindingDir}/BackwardsActionPathAux.cpp
        ${PathfindingDir}/ExperimentData.hpp
        ${PathfindingDir}/ExperimentData.cpp
        ${PathfindingDir}/Agent.hpp
        ${PathfindingDir}/Agent.cpp
        ${PathfindingDir}/Constraint.hpp
        ${PathfindingDir}/Constraint.cpp
        ${PathfindingDir}/HighLevelCBS.hpp
        ${PathfindingDir}/HighLevelCBS.cpp
        ${PathfindingDir}/LowLevelCBS.hpp
        ${PathfindingDir}/LowLevelCBS.cpp
        ${PathfindingDir}/Path.hpp
        ${PathfindingDir}/Path.cpp
        ${PathfindingDir}/Solution.hpp
        ${PathfindingDir}/Solution.cpp

        ${ControllersDir}/TestController.hpp
        ${ControllersDir}/TestController.cpp

        ${PathfindingDir}/Edge.hpp
        ${PathfindingDir}/Edge.cpp
        ${PathfindingDir}/Graph.hpp
        ${PathfindingDir}/Graph.cpp
        ${PathfindingDir}/MapStructureGraph.hpp
        ${PathfindingDir}/MapStructureGraph.cpp
        ${PathfindingDir}/Vertex.hpp
        ${PathfindingDir}/Vertex.cpp
        ${PathfindingDir}/AgentInfo.hpp
        ${PathfindingDir}/AgentInfo.cpp
        ${PathfindingDir}/Location.hpp
        ${PathfindingDir}/Location.cpp

        ${MapDir}/map_structure.hpp
        ${MapDir}/map_structure.cpp

        ${PathfindingDir}/Debugging.hpp
        ${PathfindingDir}/Logger.hpp
        ${PathfindingDir}/Logger.cpp

        ./DEFINITIONS.hpp
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