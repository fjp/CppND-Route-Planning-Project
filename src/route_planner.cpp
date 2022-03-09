#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find the closest nodes to the starting and ending coordinates.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// The distance to the end_node is used for the h value.
// Node objects have a distance method to determine the distance to another node.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// Expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto neighbor : current_node->neighbors)
    {
        // Set the parent of neighbor to current node
        neighbor->parent = current_node;

        // Set the g value to current g + distance to current
        neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
      
        // Set the h value
        neighbor->h_value = CalculateHValue(neighbor);
      
        // Add the neighbor to open_list
        open_list.emplace_back(neighbor);
      
        // Set the node's visited attribute to true
        neighbor->visited = true;
    }
}


// Sort the open list according to the sum of the h value and g value and return the next node (node with lowest sum).
RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), [](RouteModel::Node* a, RouteModel::Node* b) {
        float sum_a = a->g_value + a->h_value;
        float sum_b = b->g_value + b->h_value;
        return sum_a > sum_b;
    });
    
    RouteModel::Node* node_lowest_sum = open_list.back();
    open_list.pop_back();
  
    return node_lowest_sum;
}


// ConstructFinalPath method returns the final path found from A* search.
// - This method takes the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector has the following order: the start node is the first element
//   of the vector, the end node should is the last element.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    RouteModel::Node* node = current_node;

    while (node != start_node)
    {
        path_found.push_back(*node);
        distance += node->distance(*node->parent);
        node = node->parent;
    }
    path_found.push_back(*node);
    std::reverse(path_found.begin(), path_found.end());

    // Multiply the distance by the scale of the map to get meters.
    distance *= m_Model.MetricScale();
    return path_found;

}


// A* Search algorithm.
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    open_list.push_back(start_node);
    start_node->visited = true;
    while (open_list.size() > 0)
    {
        RouteModel::Node* next_node = NextNode();
        if (next_node == end_node)
        {
            m_Model.path = ConstructFinalPath(end_node);
            return;
        }

        AddNeighbors(next_node);
    }
}