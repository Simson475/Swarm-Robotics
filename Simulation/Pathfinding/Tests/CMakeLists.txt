include_directories(.)
include_directories(./Unit/)
include_directories(./Integration/)
include_directories(..)
include_directories(../ConstraintTree/)
include_directories(../Interfaces/)
include_directories(../../models/map/)
include_directories(../../controllers/)

set(PathfindingDir ..)
set(ControllersDir ../../controllers)
set(MapDir ../../models/map)
# We must be able to compile with these files alone (so map_structure wont work without argos files, which makes a lot of our files untestable atm)
set(PathfindingFiles
    ${PathfindingDir}/ConstraintTree/Conflict.hpp
    ${PathfindingDir}/ConstraintTree/Conflict.cpp
    ${PathfindingDir}/ConstraintTree/ConstraintTree.hpp
    ${PathfindingDir}/ConstraintTree/ConstraintTree.cpp
    ${PathfindingDir}/Action.hpp
    ${PathfindingDir}/Action.cpp
    ${PathfindingDir}/ActionPathAux.hpp
    ${PathfindingDir}/ActionPathAux.cpp
    # ${PathfindingDir}/ExperimentData.hpp
    # ${PathfindingDir}/ExperimentData.cpp
    # ${PathfindingDir}/Agent.hpp
    # ${PathfindingDir}/Agent.cpp
    ${PathfindingDir}/AgentInfo.hpp
    ${PathfindingDir}/AgentInfo.cpp
    ${PathfindingDir}/ConstraintTree/Constraint.hpp
    ${PathfindingDir}/ConstraintTree/Constraint.cpp
    ${PathfindingDir}/HighLevelCBS.hpp
    ${PathfindingDir}/HighLevelCBS.cpp
    ${PathfindingDir}/LowLevelCBS.hpp
    ${PathfindingDir}/LowLevelCBS.cpp
    ${PathfindingDir}/_path.hpp
    ${PathfindingDir}/_path.cpp
    ${PathfindingDir}/Solution.hpp
    ${PathfindingDir}/Solution.cpp
    ${PathfindingDir}/Location.hpp
    ${PathfindingDir}/Location.cpp

    # ${ControllersDir}/TestController.hpp
    # ${ControllersDir}/TestController.cpp

    ${PathfindingDir}/Interfaces/Edge.hpp
    ${PathfindingDir}/Interfaces/Edge.cpp
    ${PathfindingDir}/Interfaces/Graph.hpp
    ${PathfindingDir}/Interfaces/Graph.cpp
    ${PathfindingDir}/Interfaces/Vertex.hpp
    ${PathfindingDir}/Interfaces/Vertex.cpp

    ${PathfindingDir}/Debugging.hpp
    ${PathfindingDir}/Logger.hpp
    ${PathfindingDir}/Logger.cpp
)

set(UNIT_TESTS ./Unit)
set(INTEGRATION_TESTS ./Integration)

# Emit all warning messages
add_definitions(-Wall -O3)# -Werror

add_executable(run_tests
    
    ${PathfindingFiles}
    
    TestRunner.cpp
    TestInterface.hpp
    
    CBSTests.hpp
    CBSTests.cpp

    ${UNIT_TESTS}/ActionPathAuxTests.hpp
    ${UNIT_TESTS}/ActionPathAuxTests.cpp
    ${UNIT_TESTS}/GraphTests.hpp
    ${UNIT_TESTS}/GraphTests.cpp
    ${INTEGRATION_TESTS}/LowLevelCBSTests.hpp
    ${INTEGRATION_TESTS}/LowLevelCBSTests.cpp
    ${INTEGRATION_TESTS}/HighLevelCBSTests.hpp
    ${INTEGRATION_TESTS}/HighLevelCBSTests.cpp
    ${INTEGRATION_TESTS}/ConstraintTreeTests.hpp
    ${INTEGRATION_TESTS}/ConstraintTreeTests.cpp
    #TEST_SOURCE_FILES_LAST_LINE
    # The above line should not be edited or deleted
)

# Move the run_tests script to the external/bin/
add_custom_command(TARGET run_tests POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    ./run_tests
    ${CMAKE_BINARY_DIR}/external/bin/run_tests
    COMMENT "Copying run_tests to external/bin/"
    )