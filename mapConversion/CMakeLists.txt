cmake_minimum_required(VERSION 2.8.12)

#Project name
project (MapConversion)

#includes /Models/ directory
include_directories(Models)

#Collect all cpp files from main directory and Models/
file(GLOB SOURCES *.cpp Models/*.cpp)

link_libraries(stdc++fs)

add_executable(Main ${SOURCES})
