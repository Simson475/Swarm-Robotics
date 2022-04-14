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

int main(int argc, char *argv[]) {
    // Arrange


    // Act


    // Results

}
" > "Experiments/$1.cpp"

# Add the executable to CMakeList.txt
replaceLineExperimentExe="#NEXT_EXPERIMENT_EXECUTABLE"
sed "s/$replaceLineExperimentExe/add_executable($1_experiment \${PathfindingFiles})\n$replaceLineExperimentExe/" < "CMakeLists.txt" > "TempCMakeLists.txt"

# Add the command to move the executable to external/bin
replaceLineMoveCommand="#NEXT_EXPERIMENT_MOVE_COMMAND"
sed "s/$replaceLineMoveCommand/add_custom_command(TARGET $1_experiment POST_BUILD COMMAND \${CMAKE_COMMAND} -E copy .\/$1_experiment \${CMAKE_BINARY_DIR}\/external\/bin\/$1_experiment COMMENT \"Copying $1_experiment to external\/bin\/\" )\n$replaceLineMoveCommand/" < "TempCMakeLists.txt" > "CMakeLists.txt"
rm "TempCMakeLists.txt"
