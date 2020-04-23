cmake_minimum_required(VERSION 3.5)
project(UnitTests)

add_executable(testMain testMain.cpp )
target_link_libraries(
	testMain
	/home/widok/Desktop/Swarmrobotic/swarmrobotics/mapConversion/build/libmap_data.so
)
