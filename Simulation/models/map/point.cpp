#include "point.hpp"

unsigned int Point::id_counter = 0;

Point::Point(float x, float y, float z, pointType type, std::string name)
        : argos::CVector3(x, y, z) {

    id = id_counter++;
    this->pType = type;
    this->name = name;
}

Point::Point(CVector3 c, pointType type, std::string name)
        : CVector3(c.GetX(), c.GetY(), c.GetZ()) {
    if (pointType::tempCalculation != type)
        id = id_counter++;
    this->pType = type;
    this->name = name;
}

Point::Point() : CVector3() {
    id = id_counter++;
    this->pType = via;
}

Point::Point(Point &&p) :
        argos::CVector3(p.GetX(), p.GetY(), p.GetZ()),
        id(p.id),
        pType(p.pType),
        adjIDs(std::move(p.adjIDs)),
        name(std::move(p.name)) {
}

Point::Point(const Point &obj) :
        argos::CVector3(obj.GetX(), obj.GetY(), obj.GetZ()) {
    pType = obj.pType;
    id = obj.id;
    name = obj.name;
    adjIDs = obj.adjIDs;

}

Point &Point::operator=(const Point &obj) {
    argos::CVector3(obj.GetX(), obj.GetY(), obj.GetZ());
    pType = obj.pType;
    id = obj.id;
    name = obj.name;
    adjIDs = obj.adjIDs;
    SetX(obj.GetX());
    SetY(obj.GetY());
    SetZ(obj.GetZ());

    return *this;
}

Point::~Point() {}

void Point::setAdjIDs(std::vector<int> adjID) {
    for (auto &id : adjID) {
        adjIDs.push_back(id);
    }
}

void Point::pushAdjID(int adjID) { adjIDs.push_back(adjID); }

void Point::resetIdCount() { id_counter = 0; }

double Point::magnitude() {
    return sqrt(pow(getX(), 2) + pow(getY(), 2) + pow(getZ(), 2));
}

double Point::getX() const {
    return GetX();
}

double Point::getY() const {
    return GetY();
}

double Point::getZ() const {
    return GetZ();
}

Point Point::operator+(const Point &l) const {
    return Point(*this + l, pointType::tempCalculation, "");
}

Point Point::operator-(const Point &l) const {
    return Point(*this - l, pointType::tempCalculation, "");
}

Point Point::operator*(const Point &l) const {
    return Point(*this * l, pointType::tempCalculation, "");
}

Point Point::operator/(const Point &l) const {
    return Point(*this / l, pointType::tempCalculation, "");
}