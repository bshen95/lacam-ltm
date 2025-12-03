#pragma once

#include "graph.hpp"
#include "heap.hpp"
#include "instance.hpp"
#include "traffic_map.hpp"
#include "utils.hpp"

class ModifiedAstar
{
public:
  uint V_size;  // number of vertices
  uint W_width;
  const TrafficMap* traffic_map = nullptr;  // traffic map for edge weights
  const Graph* G = nullptr;                 // graph
  ModifiedAstar(const TrafficMap* _traffic_map, const Graph* _G);
  void setup(const TrafficMap* _traffic_map);  // initialization
  void reset(const TrafficMap* _traffic_map);
  void reset();
  inline int manhattan_dist(const Vertex* v, const Vertex* n);
  std::vector<uint> compute_traffic_path_index(uint start_id, uint end_id);
  std::vector<uint> compute_traffic_path_index_consider_past_traffic(
      uint start_id, uint end_id);

private:
  std::vector<V_Node> node_table;
  pqueue_min_f OPEN;
};