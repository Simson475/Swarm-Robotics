#ifndef CONSTRAINT_UTILS_HPP
#define CONSTRAINT_UTILS_HPP

#include "Constraint.hpp"
#include "Action.hpp"
#include "Vertex.hpp"
#include "Edge.hpp"
#include <queue>
#include "GLOBALS.hpp"

class ConstraintUtils {
public:
    static bool isViolatingConstraint(Constraint constraint, Action action);
    static bool isViolatingConstraint(Constraint constraint, std::shared_ptr<Edge> edge, float startTime);
    static bool isViolatingConstraint(Constraint constraint, std::shared_ptr<Vertex> vertex, float startTime, float endTime);
    static bool isViolatingConstraint(std::vector<Constraint> constraints, Action action);
    static Constraint getViolatedConstraint(std::vector<Constraint> constraints, Action action);
    static bool constraintCannotBeAvoided(Constraint constraint, Action action);
};

#endif