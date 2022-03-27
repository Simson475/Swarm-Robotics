#include <iostream>
#include "CBSTests.hpp"
#include "ActionPathAuxTests.hpp"

using namespace std;

int main (int argc, char *argv[]) {
    cout << "Running Tests!\n";

    ActionPathAuxTests().run();

    cout << "Testing done.\n";
} 