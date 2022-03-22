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

        ../Pathfinding/ConstraintTree/Conflict.hpp
        ../Pathfinding/ConstraintTree/Conflict.cpp
        ../Pathfinding/ConstraintTree/ConstraintTree.cpp
        ../Pathfinding/ConstraintTree/ConstraintTree.hpp
        ../Pathfinding/Action.hpp
        ../Pathfinding/Agent.cpp
        ../Pathfinding/Agent.hpp
        ../Pathfinding/ConstraintTree/Constraint.hpp
        ../Pathfinding/HighLevelCBS.cpp
        ../Pathfinding/HighLevelCBS.hpp
        ../Pathfinding/LowLevelCBS.cpp
        ../Pathfinding/LowLevelCBS.hpp
        ../Pathfinding/_path.hpp
        ../Pathfinding/_path.cpp
        ../Pathfinding/Solution.hpp
        ../Pathfinding/Solution.cpp

        TestController.hpp
        TestController.cpp

        ../Pathfinding/Interfaces/Edge.hpp
        ../Pathfinding/Interfaces/Edge.cpp
        ../Pathfinding/Interfaces/Graph.hpp
        ../Pathfinding/Interfaces/Graph.cpp
        ../Pathfinding/Interfaces/Vertex.hpp
        ../Pathfinding/Interfaces/Vertex.cpp
        ../Pathfinding/AgentInfo.hpp
        ../Pathfinding/AgentInfo.cpp

        ../Pathfinding/Debugging.hpp
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