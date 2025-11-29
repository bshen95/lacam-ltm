
#pragma once

#include "graph.hpp"
#include "heap.hpp"
#include "instance.hpp"
#include "traffic_map.hpp"
#include "utils.hpp"

struct GuidanceNode {
  int cost2goal = std::numeric_limits<int>::max();
  int cost2path = std::numeric_limits<int>::max();
};

class GuidanceHeuristic
{
public:
  const Graph* G;        // graph
  int number_of_agents;  // number of agents
  std::vector<std::vector<GuidanceNode>>
      heuristic_table;  // heuristic table for each vertex, 2-hop neighbors
  std::vector<std::queue<Vertex*>> bfs_queues;
  bool initialized = false;

  std::vector<V_Node> node_table;

  int V_size;
  int W_width;

  // for A* heuristic search
  std::vector<pqueue_min_f> OPEN;
  std::vector<std::vector<V_Node>> V_Node_table;
  std::vector<Vertex*> starts;
  std::vector<Vertex*> goals;
  const TrafficMap* traffic_map = nullptr;

  GuidanceHeuristic(const Graph* _G, const TrafficMap* _traffic_map,
                    int _number_of_agents)
      : G(_G), traffic_map(_traffic_map), number_of_agents(_number_of_agents)
  {
    heuristic_table.resize(number_of_agents);
    for (uint i = 0; i < number_of_agents; ++i) {
      heuristic_table[i].resize(G->U.size());
    }
    bfs_queues.resize(number_of_agents);
    V_size = G->V.size();
    W_width = G->width;
  }

  double manhattan_dist(const Vertex* v, const Vertex* n)
  {
    return std::abs(static_cast<int>(v->index / W_width - n->index / W_width)) +
           std::abs(static_cast<int>(v->index % W_width - n->index % W_width));
  }

  void set_traffic_map(const TrafficMap* _traffic_map)
  {
    traffic_map = _traffic_map;
  }

  void setup(const Instance* ins)
  {
    OPEN.resize(ins->N);
    starts = ins->starts;
    goals = ins->goals;
    V_Node_table =
        std::vector<std::vector<V_Node>>(ins->N, std::vector<V_Node>(V_size));
    for (size_t i = 0; i < ins->N; ++i) {
      // avoid push back copy;
      // OPEN[i] = pqueue<V_Node, cmp_less_f,min_q>(V_size);
      for (size_t j = 0; j < V_size; ++j) {
        V_Node_table[i][j].generated = false;
        V_Node_table[i][j].expanded = false;
        V_Node_table[i][j].v_id = ins->G.V[j]->id;
      }
      auto n = ins->goals[i];
      OPEN[i].push(&V_Node_table[i][n->id]);
      V_Node_table[i][n->id].g = 0;
      V_Node_table[i][n->id].h = manhattan_dist(n, starts[i]);
      V_Node_table[i][n->id].update_f();
      V_Node_table[i][n->id].generated = true;
    }
  }

  void reset(const Instance* ins)
  {
    for (size_t i = 0; i < ins->N; ++i) {
      OPEN[i].clear();
      for (size_t j = 0; j < V_size; ++j) {
        V_Node_table[i][j].expanded = false;
        V_Node_table[i][j].generated = false;
      }
      auto n = ins->goals[i];
      OPEN[i].push(&V_Node_table[i][n->id]);
      V_Node_table[i][n->id].g = 0;
      V_Node_table[i][n->id].h = manhattan_dist(n, starts[i]);
      V_Node_table[i][n->id].update_f();
      V_Node_table[i][n->id].generated = true;
    }
  }

  double get_Astar_heuristic(uint i, uint v_id)
  {
    // agent i and v_id ...
    if (V_Node_table[i][v_id].expanded) {
      return V_Node_table[i][v_id].g;
    }
    while (!OPEN[i].empty()) {
      auto& current_queue = OPEN[i];
      V_Node* curr = OPEN[i].pop();
      curr->expanded = true;
      for (auto* neighbor : G->V[curr->v_id]->neighbor) {
        uint n = neighbor->id;
        if (V_Node_table[i][n].expanded) continue;
        // auto t0 = traffic_map->get_incremental_traffic_cost(curr->v->index,
        // neighbor->index);
        auto t0 = traffic_map->get_incremental_traffic_cost(
            G->V[curr->v_id]->index, neighbor->index);
        // the free flow cost is alway one ;
        double tentative_g = curr->g + 1 + t0;
        // double tentative_g = curr->g + std::max(1.0, t0);
        // if(std::max(1.0, t0) != 1 + t0){
        //     std::cout<< " traffic cost "<< std::max(1.0, t0)<< std::endl;
        //     std::cout<< " traffic cost "<< t0 + 1.0<< std::endl;
        // }

        // auto [t0, t1] = traffic_map->get_traffic_cost(curr->v->index,
        // neighbor->index); double tentative_g = curr->g + std::max(1, t0 +
        // t1);

        double h = manhattan_dist(neighbor, starts[i]);
        if (!V_Node_table[i][neighbor->id].generated) {
          V_Node_table[i][neighbor->id].generated = true;
          V_Node_table[i][neighbor->id].g = tentative_g;
          V_Node_table[i][neighbor->id].h = h;
          V_Node_table[i][neighbor->id].f = tentative_g + h;
          OPEN[i].push(&V_Node_table[i][neighbor->id]);
        } else if (tentative_g < V_Node_table[i][neighbor->id].g) {
          V_Node_table[i][neighbor->id].g = tentative_g;
          V_Node_table[i][neighbor->id].h = h;
          V_Node_table[i][neighbor->id].f = tentative_g + h;
          OPEN[i].decrease_key(&V_Node_table[i][neighbor->id]);
        }
      }
      if (G->V[curr->v_id]->id == v_id) {
        // std::cout <<" Found heuristic for agent "<< i << " at vertex "<< v_id
        // << " with cost "<< curr->g << std::endl;
        return curr->g;
      }
    }
    // No path found
    std::cout << "No validate start and target found " << std::endl;
    return V_size;
  }

  double get_regret_heuristic(uint i, uint v_id)
  {
    // agent i and v_id ...
    if (V_Node_table[i][v_id].expanded) {
      return V_Node_table[i][v_id].g;
    }
    // std::cout <<" HHHHH I am here "<< std::endl;
    while (!OPEN[i].empty()) {
      auto& current_queue = OPEN[i];
      V_Node* curr = OPEN[i].pop();
      curr->expanded = true;
      for (auto* neighbor : G->V[curr->v_id]->neighbor) {
        uint n = neighbor->id;
        if (V_Node_table[i][n].expanded) continue;
        // auto t0 = traffic_map->get_incremental_traffic_cost(curr->v->index,
        // neighbor->index); directed graph get incoming edges. auto t0 =
        // traffic_map->get_regret_cost( curr->v->index,neighbor->index);
        auto t0 = traffic_map->get_regret_cost(neighbor->index,
                                               G->V[curr->v_id]->index);
        // auto t0 = traffic_map->get_regret_cost(curr->v->index,
        // neighbor->index); auto t0 =
        // traffic_map->get_incremental_traffic_cost(curr->v->index,
        // neighbor->index); the free flow cost is alway one ;
        double tentative_g = 0;
        // if(curr->g == 0 ){
        //     tentative_g = 1;
        // }else{
        //     tentative_g = curr->g + 1 + t0;
        // }

        // if (manhattan_dist(neighbor, goals[i]) < 5){
        //     tentative_g = curr->g + 1 ;
        // }else{
        //     tentative_g = curr->g + 1 + t0;
        // }

        // # Create Gaussian-based weight (inverted)
        // double sigma = 5;
        // double d = manhattan_dist(neighbor, goals[i]);
        // double weights = 1 - std::exp(-(d * d) / (2 * sigma * sigma));
        double weights = 1;
        tentative_g = curr->g + 1 + t0 * weights;
        // tentative_g = curr->g + 1 + t0;

        // double tentative_g = curr->g + std::max(1.0, t0);
        // if(std::max(1.0, t0) != 1 + t0){
        //     std::cout<< " traffic cost "<< std::max(1.0, t0)<< std::endl;
        //     std::cout<< " traffic cost "<< t0 + 1.0<< std::endl;
        // }

        // auto [t0, t1] = traffic_map->get_traffic_cost(curr->v->index,
        // neighbor->index); double tentative_g = curr->g + std::max(1, t0 +
        // t1);

        double h = manhattan_dist(neighbor, starts[i]);
        if (!V_Node_table[i][neighbor->id].generated) {
          V_Node_table[i][neighbor->id].generated = true;
          V_Node_table[i][neighbor->id].g = tentative_g;
          V_Node_table[i][neighbor->id].h = h;
          V_Node_table[i][neighbor->id].f = tentative_g + h;
          OPEN[i].push(&V_Node_table[i][neighbor->id]);
        } else if (tentative_g < V_Node_table[i][neighbor->id].g) {
          V_Node_table[i][neighbor->id].g = tentative_g;
          V_Node_table[i][neighbor->id].h = h;
          V_Node_table[i][neighbor->id].f = tentative_g + h;
          OPEN[i].decrease_key(&V_Node_table[i][neighbor->id]);
        }
      }
      if (G->V[curr->v_id]->id == v_id) {
        // std::cout <<" Found heuristic for agent "<< i << " at vertex "<< v_id
        // << " with cost "<< curr->g << std::endl;
        return curr->g;
      }
    }
    // No path found
    std::cout << "No validate start and target found " << std::endl;
    return V_size;
  }

  void set_gudiance_path(const std::vector<std::vector<uint>>& paths)
  {
    initialized = true;

    bfs_queues.clear();
    for (int i = 0; i < number_of_agents; ++i) {
      if (paths[i].empty()) continue;
      bfs_queues.push_back(std::queue<Vertex*>());
      for (int j = 0; j < G->U.size(); ++j) {
        heuristic_table[i][j].cost2goal = std::numeric_limits<int>::max();
        heuristic_table[i][j].cost2path = std::numeric_limits<int>::max();
      }
    }

    for (size_t i = 0; i < number_of_agents; ++i) {
      int cost2goal = 0;
      // iterate path from goal backward; skip j==0 if that's the goal itself
      // (common)
      if (paths[i].empty()) continue;
      for (size_t j = paths[i].size(); j-- > 0;) {
        // make sure include the case when path.size = 1, at goal.
        unsigned int v = paths[i][j];
        heuristic_table[i][v].cost2goal =
            cost2goal;  // distance to goal along the path from this vertex
        heuristic_table[i][v].cost2path = 0;  // on the path → zero cost to path
        bfs_queues[i].push(G->U[v]);
        ++cost2goal;
      }
    }
  }

  int get_heuristic(unsigned int agent_id, unsigned int vertex_index)
  {
    if (heuristic_table[agent_id][vertex_index].cost2path !=
        std::numeric_limits<int>::max()) {
      return heuristic_table[agent_id][vertex_index].cost2goal +
             heuristic_table[agent_id][vertex_index].cost2path;
    }
    while (!bfs_queues[agent_id].empty()) {
      Vertex* node = bfs_queues[agent_id].front();
      bfs_queues[agent_id].pop();
      int d_cost2path = heuristic_table[agent_id][node->index].cost2path;
      int d_cost2goal = heuristic_table[agent_id][node->index].cost2goal;
      for (auto neighbor : G->U[node->index]->neighbor) {
        const int d_neighbor =
            heuristic_table[agent_id][neighbor->index].cost2path;
        if (d_cost2path + 1 >= d_neighbor) continue;
        heuristic_table[agent_id][neighbor->index].cost2goal = d_cost2goal;
        heuristic_table[agent_id][neighbor->index].cost2path = d_cost2path + 1;
        bfs_queues[agent_id].push(neighbor);
      }
      if (node->index == vertex_index) {
        return heuristic_table[agent_id][vertex_index].cost2goal +
               heuristic_table[agent_id][vertex_index].cost2path;
      }
    }
  }
};