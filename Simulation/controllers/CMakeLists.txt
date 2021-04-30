include_directories(.)

# Find the QtWidgets library
find_package(Qt5Widgets) #@todo: Is this necessary anymore?
find_package(Lua)

include_directories(${LUA_INCLUDE_DIR})

#option(ARGOS_WITH_LUA "ARGOS_WITH_LUA" OFF)
#set(ARGOS_WITH_LUA "OFF")

#set(ARGOS_WITH_LUA OFF)
#add_definitions(-DARGOS_WITH_LUA=0)
unset(ARGOS_WITH_LUA CACHE)


## ********************************* SingleThreadUppaalBot
add_library(BotImplementations MODULE
        ./exceptions/StrategySynthesisError.hpp
        ./exceptions/StrategySynthesisError.cpp

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
        )

add_dependencies(BotImplementations argos-ext)

#Because argos3 installation puts in its own folder.
target_link_directories(BotImplementations PRIVATE ${EXTERNAL_INSTALL_LOCATION}/lib/argos3)

target_link_libraries(BotImplementations
        Qt5::Widgets
        argos3core_simulator
        argos3plugin_simulator_entities
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot
        argos3plugin_simulator_qtopengl
        )

add_custom_command(TARGET BotImplementations POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        ./libBotImplementations.so
        ../external/bin/libBotImplementations.so
        )

add_custom_command(TARGET BotImplementations POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        ../../controllers/parsing/planning_blueprint.xml
        ${CMAKE_BINARY_DIR}/external/bin/planning_blueprint.xml
        )