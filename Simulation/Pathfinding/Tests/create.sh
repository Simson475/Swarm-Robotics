#!/bin/bash

# $1 is the directory to put the test in
# $2 is the name of the class to test
if [ ! $# -eq 2 ]
then
echo "Missing arguments. Expected format is:"
echo "./create.sh TEST_FOLDER CLASS_TO_TEST"
exit 0
fi

# If the test directory does not exist
if [ ! -d "$1" ]
then
echo "Directory does not exist"
fi

# If the test already exists
if [ -f "$1/$2Tests.hpp" ]
then
echo "Test already exists"
exit 0
fi

# Create the header file
echo "#ifndef ${2^^}_HPP
#define ${2^^}_HPP

#include \"TestInterface.hpp\"
#include \"$2.hpp\"

class $2Tests : public TestInterface{
private:

public:
    void run(){// This is placed in the header to help ensure all tests are run

    }
}
#endif" > "$1/$2Tests.hpp"

# Create the cpp file
echo "#include \"$2Tests.hpp\"
" > "$1/$2Tests.cpp"

# Add the files to CMakeList.txt
replaceLine="    #TEST_SOURCE_FILES_LAST_LINE"
case "$1" in
 Unit) fileDir='${UNIT_TESTS}' ;;
 Integration) fileDir='${INTEGRATION_TESTS}' ;;
esac
sed "s/$replaceLine/    $fileDir\/$2Tests.hpp\n    $fileDir\/$2Tests.cpp\n$replaceLine/" < "CMakeLists.txt" > "TempCMakeLists.txt"
rm "CMakeLists.txt"
mv "TempCMakeLists.txt" "CMakeLists.txt"

# Add header in TestRunner.cpp
replaceLine='\/\/NEXT_TEST_HEADER'
sed "s/$replaceLine/#include \"$2Tests.hpp\"\n$replaceLine/" < "TestRunner.cpp" > "TempTestRunner.cpp"

# Add test "runner" in TestRunner.cpp
replaceLine='    \/\/NEXT_TEST_RUNNER'
sed "s/$replaceLine/    $2Tests().run();\n$replaceLine/" < "TempTestRunner.cpp" > "TestRunner.cpp"
rm "TempTestRunner.cpp"
