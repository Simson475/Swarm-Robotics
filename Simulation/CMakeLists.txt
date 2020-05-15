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
            URL https://github.com/ilpincy/argos3/archive/3.0.0-beta56.zip
            URL_HASH SHA512=516864a641cd3222e4260bc3c6dd09939ec9e26a19eb05b874d7ba3dd972ab91d1518d1a735de9265c88d508e3a6177d9978980df05aadcd3602181e19a87ab7
            SOURCE_SUBDIR = ../argos-ext/src
            CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EXTERNAL_INSTALL_LOCATION}
            )

    ExternalProject_Add_Step(argos-ext softLinking
            COMMAND ln -s <SOURCE_DIR>/doc <SOURCE_DIR>/../doc
            DEPENDEES build
            ALWAYS 0)

    ExternalProject_Add_Step(argos-ext makeDocs
            COMMAND cd <BINARY_DIR> && make doc
            DEPENDEES softLinking
            DEPENDERS install
            ALWAYS 1)

    include_directories(${EXTERNAL_INSTALL_LOCATION}/include)
    link_directories(${EXTERNAL_INSTALL_LOCATION}/lib)
  endif (GetDependencies)
  
project(SwarmSimulator)
set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_SOURCE_DIR})

# Emit all warning messages
add_definitions(-Wall)


# Set up C++ link directoriesemacs 

#link_directories(${ARGOS_LIBRARY_DIRS})

#message( ${CMAKE_CURRENT_SOURCE_DIR} )
#message( ${ARGOS_LIBRARY_DIRS} ) 


add_subdirectory(controllers)

add_subdirectory(loop_functions)

