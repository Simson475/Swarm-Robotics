include_directories(./Experiments/)
include_directories(.)
include_directories(..)

set(PathfindingDir ..)
# We must be able to compile with these files alone (so map_structure wont work without argos files, which makes a lot of our files untestable atm)
set(PathfindingFiles
    ${PathfindingDir}/Conflict.hpp
    ${PathfindingDir}/Conflict.cpp
    ${PathfindingDir}/ConstraintTree.hpp
    ${PathfindingDir}/ConstraintTree.cpp
    ${PathfindingDir}/Action.hpp
    ${PathfindingDir}/Action.cpp
    ${PathfindingDir}/ActionPathAux.hpp
    ${PathfindingDir}/ActionPathAux.cpp
    ${PathfindingDir}/AgentInfo.hpp
    ${PathfindingDir}/AgentInfo.cpp
    ${PathfindingDir}/Constraint.hpp
    ${PathfindingDir}/Constraint.cpp
    ${PathfindingDir}/HighLevelCBS.hpp
    ${PathfindingDir}/HighLevelCBS.cpp
    ${PathfindingDir}/LowLevelCBS.hpp
    ${PathfindingDir}/LowLevelCBS.cpp
    ${PathfindingDir}/Path.hpp
    ${PathfindingDir}/Path.cpp
    ${PathfindingDir}/Solution.hpp
    ${PathfindingDir}/Solution.cpp
    ${PathfindingDir}/Location.hpp
    ${PathfindingDir}/Location.cpp
    ${PathfindingDir}/Edge.hpp
    ${PathfindingDir}/Edge.cpp
    ${PathfindingDir}/Graph.hpp
    ${PathfindingDir}/Graph.cpp
    ${PathfindingDir}/Vertex.hpp
    ${PathfindingDir}/Vertex.cpp

    ${PathfindingDir}/Debugging.hpp
    ${PathfindingDir}/Logger.hpp
    ${PathfindingDir}/Logger.cpp
)

# Emit all warning messages
add_definitions(-Wall -O3)# -Werror

# Add the experiment executables
add_executable(chokepoint_experiment ./Experiments/chokepoint.cpp ${PathfindingFiles})
#NEXT_EXPERIMENT_EXECUTABLE
# The above line should not be edited or deleted

# Move the experiment executables to the external/bin/
add_custom_command(TARGET chokepoint_experiment POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy ./chokepoint_experiment ${CMAKE_BINARY_DIR}/external/bin/chokepoint_experiment COMMENT "Copying chokepoint_experiment to external/bin/" )
#NEXT_EXPERIMENT_MOVE_COMMAND
# The above line should not be edited or deleted

add_custom_target(move_run_experiments_script ALL
    COMMAND ${CMAKE_COMMAND} -E copy
    ../../../Pathfinding/Experiments/run_experiments.sh
    ${CMAKE_BINARY_DIR}/external/bin/run_experiments.sh)