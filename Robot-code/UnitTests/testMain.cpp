#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "json.hpp"
#include <cmath>
#include <iostream>
#include <dirent.h>
#include <sys/types.h>
#include "../map_elements.h"

using namespace std;


TEST_CASE("creation of point", "[base elements]"){
    Point p = Point(1,2,0,Type::station,std::string("P01"), nullptr);
    REQUIRE( p.getId() == -1 );
    REQUIRE( p.getX() == 1 );
    REQUIRE( p.getY() == 2 );
    REQUIRE( p.getType() == Type::station );
}
