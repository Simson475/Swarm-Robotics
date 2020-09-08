cmake_minimum_required(VERSION 3.7)

link_libraries(stdc++fs)

file(GLOB SOURCES *.cpp *.hpp)

add_executable(Server ${SOURCES})
add_dependencies(Server json-ext)
target_include_directories(Server PRIVATE ${YOUR_DIRECTORY})

add_custom_command(TARGET Server POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
                ../../server/station_scheduling.q
                ${CMAKE_CURRENT_BINARY_DIR}/station_scheduling.q
        )

add_custom_command(TARGET Server POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        ../../server/station_scheduling.xml
        ${CMAKE_CURRENT_BINARY_DIR}/station_scheduling.xml
        )

add_custom_command(TARGET Server POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        ../../server/waypoint_scheduling.xml
        ${CMAKE_CURRENT_BINARY_DIR}/waypoint_scheduling.xml
        )

add_custom_command(TARGET Server POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory
        Library
        )

add_custom_command(TARGET Server POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        ../external/lib/libconfig.so
        ${CMAKE_CURRENT_BINARY_DIR}/Library/libconfig.so
        )