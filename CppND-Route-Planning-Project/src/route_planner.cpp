#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &model.FindClosestNode(start_x, start_y);
    end_node = &model.FindClosestNode(end_x, end_y);

    std::cout<<"the value of float start_x " << start_x << "\n";
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {

float distance_h;
distance_h = (*end_node).distance(*node);

return distance_h;
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

current_node ->FindNeighbors();

for(auto neighbor:current_node ->neighbors){
    
    //Setting the parent node to current node
    neighbor->parent = current_node;

    
    
    // Calculate h and g values.
    float h_v = RoutePlanner::CalculateHValue(neighbor);
    float g_v = current_node->g_value + current_node->distance(*neighbor);

    neighbor ->h_value = h_v;
    neighbor->g_value = g_v;
    
    //Adding the neighbor to the open list
    open_list.push_back(neighbor);

    //Setting visited attribute to true
    neighbor -> visited = true; 
}


}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {

std::vector<float> f_vector;

for (int i = 0; i < open_list.size(); i++)
{
    // Calculate the f values in the vector f;
    float f_value = (open_list[i] ->h_value) + (open_list[i]->g_value);
    
    // Assign all the vectors to a vector to find the minimum among them
    f_vector.push_back(f_value);
    
    
}

//Min element index in f vector 
int minElementindex = std::min_element(f_vector.begin(), f_vector.end()) - f_vector.begin() ;

auto min_element_pointer = open_list[minElementindex];
open_list.erase(open_list.begin() + minElementindex);
return min_element_pointer;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // Start the path_found vector with the addition of the end node
    path_found.push_back(*current_node);

    // TODO: Implement your solution here.
    while (current_node->parent != nullptr)
    {
        auto parent_node_ = current_node->parent;
        
        distance += current_node ->distance(*parent_node_);

        
        path_found.push_back(*parent_node_);
        
        current_node = parent_node_;
        
    }
    
    // Reversing the  vector for the correct order
    std::reverse(path_found.begin() , path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node  = nullptr;
    // TODO: Implement your solution here.
    // Use the current node to find the neighbors
    

    open_list.push_back(start_node);
    start_node->visited = true;
    
    // Initialize the current_node variables with the start_node
    current_node = start_node;
    
    while (open_list.size()>0)
    {
        // Use the current node to find the neighbors
        AddNeighbors(current_node);
        
        // Next node using open_list
        auto next_node_pointer  = NextNode();
        
        if (next_node_pointer == end_node){
            auto Final_path = ConstructFinalPath(end_node);
            m_Model.path = Final_path;

            return;
        }
        else
        {
            current_node = next_node_pointer;
        }
        

    }
    
    
}