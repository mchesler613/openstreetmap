#pragma once

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"


class RoutePlanner {
  public:
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);
    // Add public variables or methods declarations here.

    float GetDistance() const { return distance; }

    // A* Search method to return a direct path between start_node and end_node
    void AStarSearch();

  private:
    // Add private variables or methods declarations here.

    // Add each neighboring Node to the open_list
    void AddNeighbors(RouteModel::Node *);

    // get the next node
    RouteModel::Node * NextNode();

    // return the h-value of the node
    float CalculateHValue(const RouteModel::Node *);

    // Takes the current_node and iteratively moves the sequence of parents, storing each node in the sequence,until the starting node is reached
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *current_node);

    RouteModel &m_Model;

    // These will point to the nodes in the model which are closest to our starting and ending points
    RouteModel::Node *start_node;
    RouteModel::Node *end_node;

    // This will hold the total distance for the route that A* search finds from start_node to end_node
    float distance;

    // open list of Nodes
    std::vector<RouteModel::Node *> open_list;

};
