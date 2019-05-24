#include "route_planner.h"
#include <algorithm>

//---------------------------------
// Public methods
//---------------------------------
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {

    // convert inputs to percentage
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);    
    end_node = &m_Model.FindClosestNode(end_x, end_y);    

    //std::cout << "start_node = ( " << start_node->x << "," << start_node->y << ")\n";
    //std::cout << "end_node = ( " << end_node->x << "," << end_node->y << ")\n";
}


// A* Search method to return a direct path between start_node and end_node
void RoutePlanner::AStarSearch()
{
    /*
    end_node->parent = start_node ;

    m_Model.path = ConstructFinalPath(end_node);
    */

    // set start_node->visited to true
    start_node->visited = true;

    // push start_node to back of open_list
    open_list.push_back(start_node);

    // initialize currnode to nullptr
    RouteModel::Node *currnode = nullptr;

    //std::cout << "AStarSearch()\n";
    //std::cout << "AStarSearch: Before: Size of open_list = " << open_list.size() << "\n";

    // while the open_list is not empty, traverse each node
    while (open_list.size() > 0)
    {

        // get the best node in the open_list that is closest to end_node
        currnode = NextNode();

        //std::cout << "AStarSearch: After NextNode(): Size of open_list = " << open_list.size() << "\n";
        //std::cout << "currnode->distance(*end_node) = " << currnode->distance(*end_node) << "\n";

        // if we have reached our destination end_node
        if (currnode->distance(*end_node) == 0)
        {
            // construct final path with currnode
            // set m_Model.path with results
            m_Model.path = ConstructFinalPath(currnode);

            //std::cout << "m_Model.path.size() = " << m_Model.path.size() << "\n";

            // exit
            return;
        }

        // add neighbors with currnode
        AddNeighbors(currnode);
    }
    return;
}

//---------------------------------
// Private methods
//---------------------------------

// Add each neighboring Node to the open_list
void RoutePlanner::AddNeighbors(RouteModel::Node *currnode)
{
    // populate currnode's neighbors vector
    currnode->FindNeighbors();

    //std::cout << "AddNeighbors::currnode->neighbors = " << currnode->neighbors.size() << "\n";
    //std::cout << "before: open_list.size = " << open_list.size() << "\n";

    for (RouteModel::Node *neighbor : currnode->neighbors)
    {
        // set the neighbor's parent to current_node
        neighbor->parent = currnode;

        // set the neighbor's g_value = currnode's g_value+ distance(currnode, neighbor)
        neighbor->g_value = currnode->g_value + currnode->distance(*neighbor);

        // set neighbor's h_value = CalculateHValue
        neighbor->h_value = CalculateHValue(neighbor);

        //std::cout << "neighbor g_value=" << neighbor->g_value <<"\n";
        //std::cout << "neighbor h_value=" << neighbor->h_value <<"\n";

        // push neighbor to back of open_list
        open_list.push_back(neighbor);

        // mark neighbor as visited
        neighbor->visited = true;

        //std::cout << "after: open_list.size = " << open_list.size() << "\n";
    }
}


// get the next closest node in the open_list
RouteModel::Node * RoutePlanner::NextNode()
{
    // sort the open_list based on the f_value=h_value+g_value
    std::sort(open_list.begin(), open_list.end(), [](const auto & _1st, const auto & _2nd) {
        return _1st->h_value + _1st->g_value < _2nd->h_value + _2nd->g_value;
    });

    // create a copy of the pointer to the Node with lowest f_value
    RouteModel::Node *lowest_node = open_list.front();

    // erase that node pointer from open_list
    open_list.erase(open_list.begin());

    // return the pointer copy
    return lowest_node;
}

// return the h-value of the node
float RoutePlanner::CalculateHValue(const RouteModel::Node *node)
{
    return node->distance(*end_node);
}

// Takes the current_node and iteratively moves the sequence of parents, storing each node in the sequence,until the starting node is reached
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
    std::vector<RouteModel::Node> path_found;

    distance = 0.0f;
    while (current_node->parent != nullptr)
    {
        // add current_node to path_found
        path_found.push_back(*current_node);

        // compute the distance between current_node and its parent node
        distance += current_node->distance(*current_node->parent);

        // update current_node to its parent
        current_node = current_node->parent;
    }

    path_found.push_back(*current_node);
    distance *= m_Model.MetricScale();

    return path_found;
}
