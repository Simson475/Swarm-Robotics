cmake_minimum_required(VERSION 3.7)

link_libraries(stdc++fs)

file(GLOB SOURCES *.cpp *.hpp)

add_executable(Server ${SOURCES})
add_dependencies(Server json-ext)
target_include_directories(Server PRIVATE ${YOUR_DIRECTORY})