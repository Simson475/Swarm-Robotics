#include "ConstraintUtils.hpp"

/**
 * PRE: The constraint and action are for the same agent
 * 
 * @param constraint 
 * @param action 
 * @return true
 * @return false 
 */
bool ConstraintUtils::isViolatingConstraint(Constraint constraint, Action action){
    if (constraint.location.type == ELocationType::EDGE_LOCATION){
        if (action.isWaitAction()) return false;
        auto edge = action.startVertex->getEdge(action.endVertex);
        return isViolatingConstraint(constraint, edge, action.timestamp);
    }
    if (action.isWaitAction()){
        return isViolatingConstraint(constraint, action.endVertex, action.timestamp, action.timestamp + action.duration + TIME_AT_VERTEX);
    }
    return isViolatingConstraint(constraint, action.endVertex, action.timestamp + action.duration, action.timestamp + action.duration + TIME_AT_VERTEX);
}

/**
 * PRE: The constraint and action are for the same agent
 * 
 * @param constraint 
 * @param action 
 * @return true
 * @return false 
 */
bool ConstraintUtils::isViolatingConstraint(Constraint constraint, std::shared_ptr<Edge> edge, float startTime){
    // Within constraint timespan
    float maxStart = std::max(constraint.timeStart, startTime);
    float minEnd = std::min(constraint.timeEnd, startTime + edge->getCost());
    bool withinConstraintsTimespan = maxStart <= minEnd;
    if (withinConstraintsTimespan == false) return false;

    // Violating constraint location
    return (constraint.location.type == ELocationType::EDGE_LOCATION
         && edge == constraint.location.edge);
}

/**
 * PRE: The constraint and action are for the same agent
 * 
 * @param constraint 
 * @param action 
 * @return true
 * @return false 
 */
bool ConstraintUtils::isViolatingConstraint(Constraint constraint, std::shared_ptr<Vertex> vertex, float startTime, float endTime){
    // Within constraint timespan
    float maxStart = std::max(constraint.timeStart, startTime);
    float minEnd = std::min(constraint.timeEnd, endTime);
    bool withinConstraintsTimespan = maxStart <= minEnd;
    if (withinConstraintsTimespan == false) return false;

    // Violating constraint location
    return (constraint.location == vertex);
}

bool ConstraintUtils::isViolatingConstraint(std::vector<Constraint> constraints, Action action){
    for(auto& constraint : constraints){
        if (isViolatingConstraint(constraint, action)){
            return true;
        }
    }
    return false;
}

/**
 * PRE: isViolatingConstraint must be true for atleast one constraint
 * 
 * @param constraints 
 * @param action 
 * @return Constraint 
 */
Constraint ConstraintUtils::getViolatedConstraint(std::vector<Constraint> constraints, Action action){
    for(auto& constraint : constraints){
        if (isViolatingConstraint(constraint, action)){
            return constraint;
        }
    }
    throw "Trying to get violated constraint, when no constraints are violated.";
}

bool ConstraintUtils::constraintCannotBeAvoided(Constraint constraint, Action action){
    if (action.isWaitAction()){
        if (action.duration == TIME_AT_GOAL){
            return ConstraintUtils::isViolatingConstraint(constraint, action.endVertex, action.timestamp, action.timestamp + action.duration + TIME_AT_VERTEX);
        }
        // We can avoid it if we can reduce the wait time and avoid violating it
        return ConstraintUtils::isViolatingConstraint(constraint, action.endVertex, action.timestamp, action.timestamp + TIME_AT_VERTEX);
    }
    // The action is an edge action

    // The constraint in on the vertex the edge action starts on
    if (constraint.location == action.startVertex){
        return ConstraintUtils::isViolatingConstraint(constraint, action.startVertex, action.timestamp, action.timestamp + TIME_AT_VERTEX);
    }
    if (constraint.location == action.endVertex){
        float arrivalTime = action.timestamp + action.duration;
        return ConstraintUtils::isViolatingConstraint(constraint, action.endVertex, arrivalTime, arrivalTime + TIME_AT_VERTEX);
    }
    if (constraint.location == action.getLocation()){
        return ConstraintUtils::isViolatingConstraint(constraint, action.getLocation().edge, action.timestamp);
    }
    
    return false;// The constraint is not on any location this action is
}