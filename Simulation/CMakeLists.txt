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
            URL https://github.com/GameMonkey/argos3/archive/v1.0.0.zip
            URL_HASH SHA512=211f3dc64aff3af210c1ad9a0732de8188b2fb505d6e040f6155ffcbda551594a6a5726b00c149a0937a9916529d590be31bd7e307de2cb7cf307516fc4db2a2
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