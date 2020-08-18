include_directories(.)

# Find the QtWidgets library
find_package(Qt5Widgets)
find_package(Lua)

include_directories(${LUA_INCLUDE_DIR})

add_library(footbot_diffusion MODULE footbot_diffusion.h footbot_diffusion.cpp ../connection/connector.hpp ../connection/connector.cpp)
add_dependencies(footbot_diffusion argos-ext)

#Because argos3 installation puts in its own folder.
target_link_directories(footbot_diffusion PRIVATE ${EXTERNAL_INSTALL_LOCATION}/lib/argos3)

target_link_libraries(footbot_diffusion
		Qt5::Widgets
		argos3core_simulator
		argos3plugin_simulator_entities
		argos3plugin_simulator_footbot
		argos3plugin_simulator_genericrobot
		argos3plugin_simulator_qtopengl
)

add_custom_command(TARGET footbot_diffusion POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        ./libfootbot_diffusion.so
        ../external/bin/libfootbot_diffusion.so
        )