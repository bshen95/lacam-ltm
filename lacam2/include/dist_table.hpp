/*
 * distance table with lazy evaluation, using BFS
 */
#pragma once

#include "graph.hpp"
#include "instance.hpp"
#include "utils.hpp"

struct DistTable {
  const uint V_size;  // number of vertices
  const uint U_size;  // totoal number of vertices in U
  std::vector<std::vector<uint> >
      table;          // distance table, index: agent-id & vertex-id
  std::vector<std::queue<Vertex*> > OPEN;  // search queue

  std::vector<bool> expended_vertex;  // closed list for BFS

  inline uint get(uint i, uint v_id);      // agent, vertex-id
  uint get(uint i, Vertex* v);             // agent, vertex

  std::vector<bool>& get_expended_vertex() { return expended_vertex; }
  DistTable(const Instance& ins);
  DistTable(const Instance* ins);

  void setup(const Instance* ins);  // initialization
};
