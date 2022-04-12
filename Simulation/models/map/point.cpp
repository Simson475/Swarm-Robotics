#include "point.hpp"

#include <utility>

unsigned int Point::id_counter = 0;

Point::Point(float x, float y, float z, pointType type, std::string name)
    : argos::CVector3(x, y, z) {

    id = id_counter++;
    this->pType = type;
    this->name = std::move(name);
}

Point::Point(CVector3 c, pointType type, std::string name)
    : CVector3(c.GetX(), c.GetY(), c.GetZ()) {
    id = id_counter++;
    this->pType = type;
    this->name = std::move(name);
}

Point::Point(CVector3 c)
    : CVector3(c.GetX(), c.GetY(), c.GetZ()) {
    this->id = std::numeric_limits<int>::max();
    this->pType = pointType::tempCalculation;
    this->name = "";
}

Point::Point() : CVector3() {
    id = id_counter++;
    this->pType = via;
}

Point::Point(Point &&p) noexcept:
    argos::CVector3(p.GetX(), p.GetY(), p.GetZ()),
    id(p.id),
    pType(p.pType),
    name(std::move(p.name)) {
}

Point::Point(const Point &obj) :
    argos::CVector3(obj.GetX(), obj.GetY(), obj.GetZ()) {
    pType = obj.pType;
    id = obj.id;
    name = obj.name;
}

Point &Point::operator=(const Point &obj) {
    argos::CVector3(obj.GetX(), obj.GetY(), obj.GetZ());
    pType = obj.pType;
    id = obj.id;
    name = obj.name;
    SetX(obj.GetX());
    SetY(obj.GetY());
    SetZ(obj.GetZ());

    return *this;
}

Point::~Point() = default;

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
    return Point(static_cast<argos::CVector3>(*this) + static_cast<argos::CVector3>(l));
}

Point Point::operator-(const Point &l) const {
    return Point(static_cast<argos::CVector3>(*this) - static_cast<argos::CVector3>(l));
}

std::string Point::getName() const { return name; }

int Point::getId() const { return id; }

bool Point::isOccupied() const { return occupied; }

Point::Point(float x, float y, float z) : argos::CVector3(x, y, z) {
    id = std::numeric_limits<int>::max();
    this->pType = pointType::tempCalculation;
    this->name = "";
}

double Point::getDistance(const Point &p) {
    return argos::Distance(*this, p);
}

void Point::setID(const int newID) { id = newID; }

void Point::setName(const std::string &newName) { name = newName; }

void Point::adjustPointToMid(const Point &p) {
    this->SetX((this->getX() + p.getX()) / 2);
    this->SetY((this->getY() + p.getY()) / 2);
    this->setName(getName() + "Merged" + p.getName());
}

int Point::getType() const{
    return this->pType;
}