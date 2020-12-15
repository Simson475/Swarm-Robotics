#ifndef POINT
#define POINT

#include "nlohmann/json.hpp"
#include "argos3/core/utility/math/vector3.h"

#include <sys/stat.h>
#include <sys/types.h>


enum pointType {
    via, endpoint, station, realCorner, cStation, tempCalculation
};

class Point : public argos::CVector3 {
    unsigned int id{};
    pointType pType;
    static unsigned int id_counter;
    std::vector<int> adjIDs;
    std::string name;
    //boolean used to see if anyone is occupying this station atm
    bool occupied = false;

public:
    Point(float x, float y, float z, pointType type, std::string name);

    Point(float x, float y, float z);

    Point(CVector3 c, pointType type, std::string name);

    ~Point();

    Point(Point &&p) noexcept;

    Point(const Point &obj);

    Point();

    //Getters
    //returns the type of a point, for more info check enum pointType
    [[nodiscard]] int getType() const;

    [[nodiscard]] std::string getName() const;

    double getDistance(const Point &p);

    [[nodiscard]] int getId() const;

    [[nodiscard]] double getX() const;

    [[nodiscard]] double getY() const;

    [[nodiscard]] double getZ() const;

    [[nodiscard]] std::vector<int> getAdjIDs() const;

    [[nodiscard]] bool isOccupied() const;
    //End of Getters

    //Setters
    void setID(const int newID);

    void setName(const std::string& newName);

    void setAdjIDs(const std::vector<int> &adjID);

    void pushAdjID(int adjID);

    static void resetIdCount();

    void setAsOccupied(){
        this->occupied = true;
    }

    void setAsAvailable(){
        this->occupied = false;
    }
    //Calculations
    double magnitude() const;
    //End Calculations

    //Operators
    Point operator+(const Point &l) const;

    Point &operator=(Point const &obj);

    Point operator-(const Point &l) const;
};

#endif