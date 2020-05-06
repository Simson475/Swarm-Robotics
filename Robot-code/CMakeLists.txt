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

    ExternalProject_add(model-utilities
            URL https://github.com/DEIS-Tools/model-utilities/archive/1.0.1.zip
            #URL_HASH SHA512=b47a07de9a071cce645a173d084df5dd31f7669154fc00f6c99e0506474d30e8376acaee1d3c79a50def4f25a36042951bfa4fca9a704687e59c368d05053158
            CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EXTERNAL_INSTALL_LOCATION}
            #BUILD_COMMAND make config
            #INSTALL_COMMAND make install config
            )

    include_directories(${EXTERNAL_INSTALL_LOCATION}/include)
    link_directories(${EXTERNAL_INSTALL_LOCATION}/lib)
endif (GetDependencies)

#Project name and version
project (MapConversion)
set(CMAKE_CXX_STANDARD 17)


#includes /Models/ directory
include_directories(Models)

#Collect all cpp files from main directory and Models/
file(GLOB SOURCES *.cpp Models/*.cpp)

link_libraries(stdc++fs)

add_executable(Main ${SOURCES})
add_dependencies(Main json-ext model-utilities)
