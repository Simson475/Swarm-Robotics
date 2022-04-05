#include <iostream>
#include "CBSTests.hpp"
#include "ActionPathAuxTests.hpp"
#include "GraphTests.hpp"
#include "LowLevelCBSTests.hpp"
#include "HighLevelCBSTests.hpp"
#include "HighLevelCBSTests.hpp"
#include "HighLevelCBSTests.hpp"
#include "HighLevelCBSTests.hpp"
#include "ConstraintTreeTests.hpp"
//NEXT_TEST_HEADER
// The above line should not be edited or deleted

using namespace std;

int main (int argc, char *argv[]) {
    cout << "Running Tests!\n";

    // ActionPathAuxTests().run();
    // GraphTests().run();
    // LowLevelCBSTests().run();
    ConstraintTreeTests().run();
    HighLevelCBSTests().run();
    //NEXT_TEST_RUNNER
    // The above line should not be edited or deleted

    cout << "Testing done!\n";
} 