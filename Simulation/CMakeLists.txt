cmake_minimum_required(VERSION 3.7)
include(ExternalProject)

set(EXTERNAL_INSTALL_LOCATION ${CMAKE_BINARY_DIR}/external)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(GetDependencies "Fetch external dependencies from web." ON)
if (GetDependencies)
    ExternalProject_add(json-ext
            URL https://github.com/nlohmann/json/archive/v3.7.3.zip
            URL_HASH SHA512=b47a07de9a071cce645a173d084df5dd31f7669154fc00f6c99e0506474d30e8376acaee1d3c79a50def4f25a36042951bfa4fca9a704687e59c368d05053158
            CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EXTERNAL_INSTALL_LOCATION} -DJSON_BuildTests=OFF
            )

    ExternalProject_add(argos-ext
            URL https://github.com/ilpincy/argos3/archive/3.0.0-beta54.zip
            URL_HASH SHA512=cbce91291e1c3d622c094a6186c31ff236f9e60f7b17dd1ba19685f360b9fe33d27d7225e6233f702dfc190ba8fb0cee8c3c352e6460b1f507295f782a78836c
            SOURCE_SUBDIR = ../argos-ext/src
            CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EXTERNAL_INSTALL_LOCATION} -DARGOS_INSTALL_LDSOCONF=OFF -DARGOS_DOCUMENTATION=OFF
            )

    ExternalProject_add(model-utilities
            URL https://github.com/DEIS-Tools/model-utilities/archive/1.0.1.zip
            CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EXTERNAL_INSTALL_LOCATION}
            )

    include_directories(${EXTERNAL_INSTALL_LOCATION}/include)
    link_directories(${EXTERNAL_INSTALL_LOCATION}/lib)
  endif (GetDependencies)


unset(ARGOS_WITH_LUA CACHE)

project(SwarmSimulator)
set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_SOURCE_DIR})

# Emit all warning messages
add_definitions(-Wall -Werror)


add_subdirectory(controllers)

add_subdirectory(loop_functions)

add_subdirectory(server)