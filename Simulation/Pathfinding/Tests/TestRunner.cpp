#include <iostream>
#include "CBSTests.hpp"
#include "ActionPathAuxTests.hpp"
#include "GraphTests.hpp"
#include "LowLevelCBSTests.hpp"

using namespace std;

int main (int argc, char *argv[]) {
    cout << "Running Tests!\n";

    ActionPathAuxTests().run();
    GraphTests().run();
    LowLevelCBSTests().run();

    cout << "Testing done!\n";
} 