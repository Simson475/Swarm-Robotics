include_directories(.)
include_directories(..)
include_directories(../ConstraintTree/)
include_directories(../Interfaces/)
include_directories(../../models/map/)
include_directories(../../controllers/)

# Emit all warning messages
add_definitions(-Wall)# -Werror

add_executable(run_tests # Name of executable
    TestRunner.cpp # Rest is source files
    
    CBSTests.cpp
    CBSTests.hpp

    ${PathfindingFiles}
)

# Move the run_a_scene.sh script to the external/bin/run_a_scene.sh
add_custom_target(move_run_tests_script ALL
        COMMAND ${CMAKE_COMMAND} -E copy
        ../../Pathfinding/Tests/run_tests
        ${CMAKE_BINARY_DIR}/external/bin/run_tests)