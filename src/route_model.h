#pragma once

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iostream>

class RouteModel : public Model {

  public:
    class Node : public Model::Node {
      public:
        // Add public Node variables and methods here.
        
        Node(){}
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}
      
        // pointer to the Node's parent
        Node *parent = nullptr;

        // the h-value for the node
        float h_value = std::numeric_limits<float>::max();

        // the g-value for the node
        float g_value = 0.0;

        // has the node been visited?
        bool visited = false;

        // this node's neighbors
        std::vector<Node *> neighbors;

        // calculate distance between two RouteModel::Node objects
        float distance(Node n) const 
        {
            return std::sqrt(
                std::pow(this->x - n.x, 2)+
                std::pow(this->y - n.y, 2)
                );
        }

        // find neighbors by populating the neighbors' vector of the current node
        void FindNeighbors();

      private:
        // find closest node in each Road containing the current node
        RouteModel::Node * FindNeighbor(std::vector<int> node_indices);

        // Add private Node variables and methods here.
        int index;
        RouteModel * parent_model = nullptr;
    };
    
    // Add public RouteModel variables and methods here.
    RouteModel(const std::vector<std::byte> &xml);  


    // This variable will eventually store the path that is found by the A* search.
    std::vector<RouteModel::Node> path;

    std::vector<RouteModel::Node> & SNodes() { return m_Nodes; }


    auto & GetNodeToRoadMap() { return node_to_road;}

    RouteModel::Node & FindClosestNode(float x, float y);    

  private:
    // Add private RouteModel variables and methods here.

    void CreateNodeToRoadHashmap();

    // store all the nodes from the Open Street Map data
    std::vector<RouteModel::Node> m_Nodes;

    // which roads does this node belong to
    std::unordered_map<int, std::vector<const Model::Road *>> node_to_road;


};
