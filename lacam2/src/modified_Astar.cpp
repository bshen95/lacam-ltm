#include "../include/modified_Astar.hpp"
#include <cstdlib>

ModifiedAstar::ModifiedAstar(const TrafficMap* _traffic_map,const Graph* _G)
    : traffic_map(_traffic_map), G(_G),V_size(_traffic_map->width * _traffic_map->height),
    W_width(_traffic_map->width)
{
    setup(_traffic_map);
}

void ModifiedAstar::setup(const TrafficMap* _traffic_map)
{   
    node_table.resize(V_size);
    // OPEN = pqueue<V_Node, cmp_less_f_tie_breaker, min_q>(V_size);
}

void ModifiedAstar::reset(const TrafficMap* _traffic_map)
{
    // reset with different traffic map; 
    traffic_map = _traffic_map;
    V_size = _traffic_map->width * _traffic_map->height;
    W_width = _traffic_map->width;
    node_table.resize(V_size);
    OPEN.clear();
    for (uint i = 0; i < V_size; ++i) {
        node_table[i].expanded = false;
        node_table[i].generated = false;
    }
}

void ModifiedAstar::reset()
{
    // reset with the same traffic map; 
    OPEN.clear();
    for (uint i = 0; i < V_size; ++i) {
        node_table[i].g = 0.0;
        node_table[i].h = 0;
        node_table[i].f = node_table[i].g + node_table[i].h;
        node_table[i].expanded = false;
        node_table[i].generated = false;
    }
}

std::vector<uint> ModifiedAstar::compute_traffic_path_index(uint start_index, uint end_index){
    reset();
    Vertex* start_vertex = G->U[start_index];
    Vertex* target_vertex = G->U[end_index];

    // Initialize start node
    node_table[start_index].g = 0.0;
    node_table[start_index].h = manhattan_dist(start_vertex, target_vertex);
    node_table[start_index].f = node_table[start_index].g + node_table[start_index].h;
    node_table[start_index].v_id = start_vertex->id;
    OPEN.push(&node_table[start_index]);
    node_table[start_index].generated = true;

    std::vector<uint> path;
    while (!OPEN.empty()) {
        V_Node* curr = OPEN.pop();
        curr->expanded = true;

        if (G->V[curr->v_id]->index== end_index) {
            // Path found, reconstruct cost
            // std::cout<< "path found"<<  node_table[curr->v->index].f << std::endl;
            uint v_index = G->V[curr->v_id]->index;
            while (v_index != start_index) {
                path.push_back(v_index);
                v_index = node_table[v_index].predecessor;
            }
            path.push_back(start_index);
            std::reverse(path.begin(), path.end()); 
            return path;
        }

        for (auto* neighbor : G->V[curr->v_id]->neighbor) {
            uint n = neighbor->index;
            if (node_table[n].expanded) continue;

            auto [t0, t1] = traffic_map->get_traffic_cost_contra_flow_version(G->V[curr->v_id]->index, n);
            // the free flow cost is alway one ;
            double tentative_g = curr->g + 1 + t0 + t1;
            // double tentative_g = curr->g + std::max(1.0,t0 + t1);
            // double tentative_g = curr->g +1;
            if (!node_table[n].generated) {
                node_table[n].generated   = true;
                node_table[n].v_id           = neighbor->id;          // or pre-fill v by index in reset()
                node_table[n].h           = manhattan_dist(neighbor, target_vertex);
                node_table[n].g           = tentative_g;
                node_table[n].predecessor = G->V[curr->v_id]->index;
                node_table[n].f           = tentative_g + node_table[n].h;
                OPEN.push(&node_table[n]);
            } else if (tentative_g < node_table[n].g) {
                node_table[n].g           = tentative_g;
                node_table[n].predecessor = G->V[curr->v_id]->index;
                node_table[n].f           = tentative_g + node_table[n].h; // reuse h
                OPEN.decrease_key(&node_table[n]);
            }
        }

    }
    // No path found
    std::cout << "No path found from " << start_index << " to " << end_index << std::endl;
    return path;

}


std::vector<uint> ModifiedAstar::compute_traffic_path_index_consider_past_traffic(uint start_index, uint end_index){
    reset();
    Vertex* start_vertex = G->U[start_index];
    Vertex* target_vertex = G->U[end_index];

    // Initialize start node
    node_table[start_index].g = 0.0;
    node_table[start_index].h = manhattan_dist(start_vertex, target_vertex);
    node_table[start_index].f = node_table[start_index].g + node_table[start_index].h;
    node_table[start_index].v_id = start_vertex->id;
    OPEN.push(&node_table[start_index]);
    node_table[start_index].generated = true;

    std::vector<uint> path;
    while (!OPEN.empty()) {
        V_Node* curr = OPEN.pop();
        curr->expanded = true;

        if (G->V[curr->v_id]->index== end_index) {
            // Path found, reconstruct cost
            uint v_index = G->V[curr->v_id]->index;
            while (v_index != start_index) {
                path.push_back(v_index);
                v_index = node_table[v_index].predecessor;
            }
            path.push_back(start_index);
            std::reverse(path.begin(), path.end()); 
            return path;
        }

        for (auto* neighbor : G->V[curr->v_id]->neighbor) {
            uint n = neighbor->index;
            if (node_table[n].expanded) continue;

            auto [t0, t1] = traffic_map->get_traffic_cost_contra_flow_version(G->V[curr->v_id]->index, n);
            auto t2 = traffic_map->get_incremental_traffic_cost(G->V[curr->v_id]->index, n);
            // auto t2 = traffic_map->get_incremental_traffic_cost(curr->v->index, n);
            // the free flow cost is alway one ;
            double tentative_g = curr->g + 1 + t0 + t1 + t2;

            if (!node_table[n].generated) {
                node_table[n].generated   = true;
                node_table[n].v_id           = neighbor->id;          // or pre-fill v by index in reset()
                node_table[n].h           = manhattan_dist(neighbor, target_vertex);
                node_table[n].g           = tentative_g;
                node_table[n].predecessor = G->V[curr->v_id]->index;
                node_table[n].f           = tentative_g + node_table[n].h;
                OPEN.push(&node_table[n]);
            } else if (tentative_g < node_table[n].g) {
                node_table[n].g           = tentative_g;
                node_table[n].predecessor = G->V[curr->v_id]->index;
                node_table[n].f           = tentative_g + node_table[n].h; // reuse h
                OPEN.decrease_key(&node_table[n]);
            }
        }

    }
    // No path found
    std::cout << "No path found from " << start_index << " to " << end_index << std::endl;
    return path;

}


int ModifiedAstar::manhattan_dist(const Vertex* v, const Vertex* n){
  return std::abs(static_cast<int>(v->index / W_width - n->index / W_width)) + 
         std::abs(static_cast<int>(v->index % W_width - n->index % W_width));
}

