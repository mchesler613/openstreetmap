#include "route_model.h"
#include <iostream>

//----------------------------------------
//  Public Methods
//----------------------------------------
RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
    int counter=0;
    for (Model::Node node : this->Nodes())
    {
        m_Nodes.push_back(Node(counter, this, node));
        counter++;
    }

    CreateNodeToRoadHashmap();
}

void RouteModel::CreateNodeToRoadHashmap()
{
    // write a loop that iterates through the vector given by calling Roads()
   // for each reference &road, check that the type is not a footway
   // for (const Model::Road &road : Roads())
   for (const Model::Road &road : Roads())
   {
        if (road.type != Model::Road::Type::Footway)
        {
            // loop over each node_idx in the way that the road belongs to:
            for (int node_idx :  Ways()[road.way].nodes)
            {
               // if node index is not in the node_to_road hashmap yet, set the value for the node_idx key to be an empty vector of const Model::Road* objects

               if (node_to_road.find(node_idx) ==  node_to_road.end())
                {
                    node_to_road[node_idx] = std::vector<const Model::Road *>(); 
                }

                // push a pointer to the current road to the back of the vector given by the node_idx key in node_to_read
                node_to_road[node_idx].push_back(&road);
            }
        }
   } 
}



//----------------------------------------
// find neighbors by populating the neighbors'
// vector of the current node
//----------------------------------------
void RouteModel::Node::FindNeighbors()
{
    for (auto &road : parent_model->node_to_road[this->index])
    {
        RouteModel::Node *closestnode = this->FindNeighbor(parent_model->Ways()[road->way].nodes);

        if (closestnode != nullptr)
        {
            this->neighbors.push_back(closestnode);
        }
    }
}


//----------------------------------------
// Find the closest Node based on (x,y) location
//----------------------------------------

RouteModel::Node & RouteModel::FindClosestNode(float x, float y)
{
    RouteModel::Node currnode;
    currnode.x = x;
    currnode.y = y;

    float min_dist = std::numeric_limits<float>::max();
    float dist;
    int closest_idx;

    for (const Model::Road &road : Roads())
    {
        if (road.type != Model::Road::Type::Footway)
        {
            for (int node_idx :  Ways()[road.way].nodes)
            {
                dist = currnode.distance(SNodes()[node_idx]);
                if (dist < min_dist)
                {
                    closest_idx = node_idx;
                    min_dist = dist;
                }
            }
        }
    }

    return SNodes()[closest_idx];
}

//----------------------------------------
//  Private Methods
//----------------------------------------

// find closest node in each Road containing the current node
RouteModel::Node * RouteModel::Node::FindNeighbor(std::vector<int> node_indices)
{
    // initialize the closest_node to nothing
    Node *closest_node = nullptr;
    Node node;

    for (int node_index : node_indices)
    {
       // retrieve the current Node object from the index
       node = parent_model->SNodes()[node_index];

       // if distance(this,node) is nonzero
       // and the node has not been visited
        if (this->distance(node) != 0 && !node.visited)
        {
          
            if (closest_node == nullptr || this->distance(node) < this->distance(*closest_node))
            {
                closest_node = &parent_model->SNodes()[node_index];
            }
           
        }
    }

    return closest_node;
}
