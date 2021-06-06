#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;


    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);

}




float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
     return node->distance(*end_node);

}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for(auto node : current_node->neighbors){
        node->parent = current_node;
        node ->g_value = current_node->g_value + current_node->distance(*node);
        node-> h_value = CalculateHValue(node);
        open_list.push_back(node);
        node->visited = true;

    }
}



RouteModel::Node *RoutePlanner::NextNode() {
     std::sort(open_list.begin(), open_list.end(), [](RouteModel::Node *a, RouteModel::Node * b){
        return (a->h_value + a->g_value) > (b->h_value +b->g_value);
    });
    
    RouteModel::Node *lowest_sum_node = open_list.back();
    open_list.pop_back();
   
    return lowest_sum_node;

}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    

    while (current_node -> parent !=nullptr){

        path_found.push_back(*current_node);
        RouteModel::Node* node = current_node->parent;
        distance += current_node -> distance(*node);
        current_node = node;
    }
    path_found.push_back(*current_node);
    std::reverse(path_found.begin(),path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}



void RoutePlanner::AStarSearch() {
    
    start_node ->visited =true;
    open_list.push_back(start_node);
    RouteModel::Node *current_node = nullptr;

    while(!open_list.empty()) {

        current_node = NextNode();

        if (current_node ->distance(*end_node) == 0){
            m_Model.path = ConstructFinalPath(current_node);
           
        }
        AddNeighbors(current_node); 
    }

}