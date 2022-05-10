#!/bin/bash

# $1 is the name of the experiment
if [ ! $# -eq 1 ]
then
echo "Missing arguments. Expected format is:"
echo "./create.sh EXPERIMENT_NAME"
exit 0
fi

# If the experiment already exists
if [ -f "Experiments/$1.cpp" ]
then
echo "Experiment already exists"
exit 0
fi

# Create the cpp file
echo "#include \"HighLevelCBS.hpp\"
#include <sys/stat.h>

int main(int argc, char *argv[]) {
    Logger& logger = Logger::get_instance();
    // Create folder for results
    std::string experimentResultDir = \"${1}_experiment_result\";
    mkdir(&experimentResultDir[0], 0777);

    for (int agentCount = 1; agentCount <= 15; ++agentCount){
        std::cout << \"Running experiment with \" << agentCount << \" agents..\";
        std::cout.flush();
        std::string experimentResultFile = experimentResultDir + \"/\" + std::to_string(agentCount) + \"agents_${1}.txt\";
        // Remove any existing results if they exist
        remove(&experimentResultFile[0]);
        logger.setLogFile(experimentResultFile);

        // Arrange experiment
        //TODO construct the graph and agents...
        // Create vertices
        int vertexCount = 1; //TODO actual vertex count
        std::vector<std::shared_ptr<Vertex>> vertices(vertexCount);
        for (int i = 0; i < vertexCount; ++i){
            vertices[i] = std::make_shared<Vertex>(i);
        }
        //TODO create edges
        //TODO create graph
        std::vector<AgentInfo> agents(agentCount);
        for (int i = 0; i < agentCount; ++i){
            auto startVertex = vertices[i];
            auto goalVertex = vertices[i];// TODO set to actual goal
            agents[i] = AgentInfo(i, Action(0, startVertex, startVertex, 0), goalVertex);
        }

        // Act
        std::chrono::steady_clock::time_point experimentBeginTime = std::chrono::steady_clock::now();
        Solution solution = HighLevelCBS::get_instance().findSolution(graph, agents, LowLevelCBS::get_instance());
        auto experimentTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - experimentBeginTime).count();

        // Results
        (*logger.begin()) << \"Experiment took \"<< experimentTime << \"[µs]\n\"; logger.end();
        std::cout << \"Done\n\";
    }
}
" > "Experiments/$1.cpp"

# Add the executable to CMakeList.txt
replaceLineExperimentExe="#NEXT_EXPERIMENT_EXECUTABLE"
sed "s/$replaceLineExperimentExe/add_executable($1_experiment .\/Experiments\/$1.cpp \${PathfindingFiles})\n$replaceLineExperimentExe/" < "CMakeLists.txt" > "TempCMakeLists.txt"

# Add the command to move the executable to external/bin
replaceLineMoveCommand="#NEXT_EXPERIMENT_MOVE_COMMAND"
sed "s/$replaceLineMoveCommand/add_custom_command(TARGET $1_experiment POST_BUILD COMMAND \${CMAKE_COMMAND} -E copy .\/$1_experiment \${CMAKE_BINARY_DIR}\/external\/bin\/$1_experiment COMMENT \"Copying $1_experiment to external\/bin\/\" )\n$replaceLineMoveCommand/" < "TempCMakeLists.txt" > "CMakeLists.txt"
rm "TempCMakeLists.txt"
