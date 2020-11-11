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

## ********************************* CFootBotDiffusion
add_library(CFootBotDiffusion MODULE
        CFootBotDiffusion.h
        CFootBotDiffusion.cpp
        ../connection/connector.hpp
        ../connection/connector.cpp)

add_dependencies(CFootBotDiffusion argos-ext)

#Because argos3 installation puts in its own folder.
target_link_directories(CFootBotDiffusion PRIVATE ${EXTERNAL_INSTALL_LOCATION}/lib/argos3)

target_link_libraries(CFootBotDiffusion
		Qt5::Widgets
		argos3core_simulator
		argos3plugin_simulator_entities
		argos3plugin_simulator_footbot
		argos3plugin_simulator_genericrobot
		argos3plugin_simulator_qtopengl
)

add_custom_command(TARGET CFootBotDiffusion POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        ./libCFootBotDiffusion.so
        ../external/bin/libCFootBotDiffusion.so
        )


## ********************************* SingleThreadUppaalBot
add_library(SingleThreadUppaalBot MODULE
        SingleThreadUppaalBot.hpp
        SingleThreadUppaalBot.cpp)

add_dependencies(SingleThreadUppaalBot argos-ext)

target_link_directories(SingleThreadUppaalBot PRIVATE ${EXTERNAL_INSTALL_LOCATION}/lib/argos3)

target_link_libraries(SingleThreadUppaalBot
        Qt5::Widgets
        argos3core_simulator
        argos3plugin_simulator_entities
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot
        argos3plugin_simulator_qtopengl
        )

add_custom_command(TARGET SingleThreadUppaalBot POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        ./libSingleThreadUppaalBot.so
        ../external/bin/libSingleThreadUppaalBot.so
        )