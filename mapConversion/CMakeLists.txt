cmake_minimum_required(VERSION 2.8.12)

project (MapConversion)


add_library(map_data MODULE 
	Main.cpp
	map_elements.h
	map_elements.cpp
	map_structure.cpp
	map_structure.h
	stratego.cpp
	stratego.h
	gnuplot.h
)

link_libraries(stdc++fs)

target_link_libraries(map_data)
add_executable(Main Main.cpp map_elements.cpp map_structure.cpp stratego.cpp)
