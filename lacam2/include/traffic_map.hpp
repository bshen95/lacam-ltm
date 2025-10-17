
#pragma once
#include <vector>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
typedef unsigned int uint;
struct EdgePairHash {
    size_t operator()(const std::pair<int,int>& p) const {
        return std::hash<long long>()(((long long)p.first << 32) ^ p.second);
    }
};

struct TrafficMap {
    int width, height;
    // vertex flow && contra flow 
    std::vector<double> vertex_flow;
    std::vector<double> edge_flow;
    const Graph* G = nullptr;
    // traffic map takes vertex->index as input !
    std::vector<double> incremental_flow;
    std::vector<bool> visited;
    std::vector<double> normalized_incremental_flow;
    std::vector<double> PIBT_edge_Regret;
    std::vector<double> PIBT_vertex_Regret;
    std::vector<double> PIBT_regret_flow;
    std::vector<double> normalized_PIBT_regret_flow;

    double scaling_factor = 10;
    double scaling_min = 0;
    double scaling_max = 100;

    TrafficMap(const Graph* _G) : G(_G) , width(_G->width), height(_G->width) {
      // int total_edges = (width - 1) * height + width * (height - 1);
      // edge_flow.resize(total_edges, 0.0);
      incremental_flow.resize(width * height * 4, 0);
      normalized_incremental_flow.resize(width * height * 4, 0);
      edge_flow.resize(width * height * 4, 0);
      vertex_flow.resize(width * height, 0);
      
      visited.resize(G->U.size(), false);

      PIBT_regret_flow.resize(width * height * 5, 0);
      normalized_PIBT_regret_flow.resize(width * height * 5, 0);
      PIBT_edge_Regret.resize(width * height * 5, 0);
      PIBT_vertex_Regret.resize(width * height, 0);

      // std::cout<< total_edges <<std::endl;
    }

    int edge_index(uint a, uint b) const {
      // For a grid: each vertex has up to 4 outgoing edges (right, left, down, up)
      // Assign each direction a slot: 0=right, 1=left, 2=down, 3=up
      int ax = a % width, ay = a / width;
      int bx = b % width, by = b / width;
      int dir = -1;
      if (bx == ax + 1 && by == ay) dir = 0;      // right
      else if (bx == ax - 1 && by == ay) dir = 1; // left
      else if (bx == ax && by == ay + 1) dir = 2; // down
      else if (bx == ax && by == ay - 1) dir = 3; // up
      if (dir == -1) {
        std::cout<< "This should never happened" <<std::endl;
        return -1;
      }
      return a * 4 + dir;
    }

    int edge_waiting_index(uint a, uint b) const {
      // For a grid: each vertex has up to 5 outgoing edges 
      // (right, left, down, up, wait)
      // Assign each direction a slot: 
      // 0 = right, 1 = left, 2 = down, 3 = up, 4 = wait

      int ax = a % width, ay = a / width;
      int bx = b % width, by = b / width;
      int dir = -1;

      if (bx == ax + 1 && by == ay) dir = 0;      // right
      else if (bx == ax - 1 && by == ay) dir = 1; // left
      else if (bx == ax && by == ay + 1) dir = 2; // down
      else if (bx == ax && by == ay - 1) dir = 3; // up
      else if (bx == ax && by == ay) dir = 4;     // wait (no movement)

      if (dir == -1) {
          std::cerr << "Invalid edge between " << a << " and " << b << std::endl;
          return -1;
      }

      return a * 5 + dir;
    }
    void increase_regret_on_edge(uint a, uint b, double regret_value) {
      visited[a] = true;
      visited[b] = true;
      if(a == b){
        PIBT_vertex_Regret[b] += regret_value;
      }
      int edge_idx = edge_waiting_index(a, b);
      PIBT_edge_Regret[edge_idx] += regret_value;
    } 
    

    


    std::tuple<double,double> get_recorded_regret_cost (uint a, uint b) const {
      if(a == b) return {0, 0};
      // if(a == b) return {0, 0}; // No cost if the same vertex
      int edge_idx = edge_waiting_index(a, b);
      int edge_idx2 = edge_waiting_index(b, a);
      // return { std::sqrt(( edge_flow[edge_idx] + 1) * edge_flow[edge_idx2]), (vertex_flow[b]) / 2 };
      //  return { edge_flow[edge_idx] ,  (vertex_flow[b])/2};
      // return { edge_flow[edge_idx],  (vertex_flow[b])/2};
      return { PIBT_edge_Regret[edge_idx],  (PIBT_vertex_Regret[b])};
    }



    void reset_regret_cost(){
      std::fill(visited.begin(), visited.end(), false);
      std::fill(PIBT_vertex_Regret.begin(), PIBT_vertex_Regret.end(), 0);
      std::fill(PIBT_edge_Regret.begin(), PIBT_edge_Regret.end(), 0);
      // std::fill(normalized_PIBT_regret_flow.begin(), normalized_PIBT_regret_flow.end(), 0);
    } 

    double get_regret_cost(uint a, uint b) const {
      // if(a == b) return 0;
      int edge_idx = edge_waiting_index(a, b);
      // return incremental_flow[edge_idx];
      return scaling_factor*normalized_PIBT_regret_flow[edge_idx];
    }


    void increase_regret_based_on_path(const std::vector<uint>& path) {
      // remove path from traffic map
      for (size_t i = 0; i < path.size() - 1; ++i) {
        visited[path[i]] = true;
        visited[path[i+1]] = true;
        uint a = path[i];
        uint b = path[i + 1];
        // PIBT_vertex_Regret[b] ++;
        // if(a == b) {
        //   continue;}
        // int e_index = edge_index(a, b);
        // PIBT_edge_Regret[e_index] ++; 
        if(a == b) {
          PIBT_vertex_Regret[b] ++;
        } // Skip if the same vertex
        int e_index = edge_waiting_index(a, b);
        PIBT_edge_Regret[e_index] ++; 
      }
    }

    void record_regret_cost( double learning_rate){
      std::unordered_set<std::pair<int,int>, EdgePairHash> visited_edge;
      for(size_t i = 0; i < visited.size(); ++i) {
        if(visited[i]) {
          for(const auto& neighbor : G->U[i]->neighbor){
            visited_edge.insert({i, neighbor->index});
            visited_edge.insert({neighbor->index, i});
          }
          visited_edge.insert({i, i}); // self-loop for vertex cost
        }
      }
      // std::fill(incremental_flow.begin(), incremental_flow.end(), 0);
      for(auto& edge : visited_edge){
        // if( edge.first == edge.second) continue;
        int edge_idx = edge_waiting_index(edge.first, edge.second);
        int revserse_edge_idx = edge_waiting_index( edge.second, edge.first);
        // auto [t1, t2] = get_recorded_regret_cost(edge.first, edge.second);
        // incremental_flow[edge_idx] += learning_rate*(t1 + t2 );
        if( edge.first == edge.second){
          PIBT_regret_flow[edge_idx] +=  PIBT_edge_Regret[edge_idx];
        }else{
          PIBT_regret_flow[edge_idx] += PIBT_edge_Regret[revserse_edge_idx] + PIBT_vertex_Regret[edge.first];
        }
        // incremental_flow[edge_idx] += (t1 + t2);
        // std::cout<< " Adding regret cost on edge from "<< edge.first << " to "<< edge.second << " with cost "
        // << (t1 + t2) << std::endl;
      }
      double max_val = 0.0;
      for (double val : PIBT_regret_flow) {
          if (val > max_val) max_val = val;
      }
      if (max_val > 0) {
        for(int i = 0; i < normalized_PIBT_regret_flow.size(); ++i){
          normalized_PIBT_regret_flow[i] = PIBT_regret_flow[i] / max_val;
        }
      }
      
      
    }
        
    void reset() {
      std::fill(visited.begin(), visited.end(), false);
      std::fill(vertex_flow.begin(), vertex_flow.end(), 0);
      std::fill(edge_flow.begin(), edge_flow.end(), 0);
      std::fill(normalized_incremental_flow.begin(), normalized_incremental_flow.end(), 0);
    }
    
    // int undirected_edge_index(uint a, uint b) const {
    //   if (a > b) std::swap(a, b);
    //   int ax = a % width, ay = a / width;
    //   int bx = b % width, by = b / width;

    //   if (bx == ax + 1 && by == ay) {  // horizontal edge
    //       // First all horizontal edges in this row
    //       return ay * (2*width-1) + ax;
    //   } else if (by == ay + 1 && bx == ax) {  // vertical edge
    //       // Then vertical edges in this row
    //       return ay * (2*width-1) + (width-1) + ax;
    //   }else {
    //       std::cout<< "This should never happened" <<std::endl;
    //       return -1; // not adjacent
    //   }
    // }

    // int edge_index(uint a, uint b) const {
    //   if (a > b) std::swap(a, b);
    //   int ax = a % width, ay = a / width;
    //   int bx = b % width, by = b / width;

    //   if (bx == ax + 1 && by == ay) {  // horizontal edge
    //       // First all horizontal edges in this row
    //       return ay * (2*width-1) + ax;
    //   } else if (by == ay + 1 && bx == ax) {  // vertical edge
    //       // Then vertical edges in this row
    //       return ay * (2*width-1) + (width-1) + ax;
    //   }else {
    //       std::cout<< "This should never happened" <<std::endl;
    //       return -1; // not adjacent
    //   }
    // }

    double get_incremental_traffic_cost(uint a, uint b) const {
      if(a == b) return 0;
      int edge_idx = edge_index(a, b);
      // return incremental_flow[edge_idx];
      return scaling_factor*normalized_incremental_flow[edge_idx];
    }

    std::tuple<double,double> get_traffic_cost_contra_flow_version(uint a, uint b) const {
      if(a == b) return {0, 0};
      // if(a == b) return {0, 0}; // No cost if the same vertex
      int edge_idx = edge_index(a, b);
      int edge_idx2 = edge_index(b, a);
      // return { std::sqrt(( edge_flow[edge_idx] + 1) * edge_flow[edge_idx2]), (vertex_flow[b]) / 2 };
      //  return { edge_flow[edge_idx] ,  (vertex_flow[b])/2};
      // return { edge_flow[edge_idx],  (vertex_flow[b])/2};
      return { (edge_flow[edge_idx] + 1) * (edge_flow[edge_idx2]) ,  (vertex_flow[b])/2};
    }
    
    std::tuple<double,double> get_traffic_cost(uint a, uint b) const {
      if(a == b) return {0, 0};
      // if(a == b) return {0, 0}; // No cost if the same vertex
      int edge_idx = edge_index(a, b);
      int edge_idx2 = edge_index(b, a);
      // return { std::sqrt(( edge_flow[edge_idx] + 1) * edge_flow[edge_idx2]), (vertex_flow[b]) / 2 };
      //  return { edge_flow[edge_idx] ,  (vertex_flow[b])/2};
      // return { edge_flow[edge_idx],  (vertex_flow[b])/2};
      return { edge_flow[edge_idx],  (vertex_flow[b])};
    }


    void remove_path(const std::vector<uint>& path) {
      // remove path from traffic map
      for (size_t i = 0; i < path.size() - 1; ++i) {
        uint a = path[i];
        uint b = path[i + 1];
        vertex_flow[b] --; 
        if(a == b) continue; // Skip if the same vertex
        int e_index = edge_index(a, b);
        edge_flow[e_index] --; 
      }
    }

    void add_path(const std::vector<uint>& path) {
      // remove path from traffic map
      for (size_t i = 0; i < path.size() - 1; ++i) {
        uint a = path[i];
        uint b = path[i + 1];
        vertex_flow[b] ++;
        if(a == b) continue; // Skip if the same vertex
        int e_index = edge_index(a, b);
        edge_flow[e_index] ++; 
      }
    }


    void add_incremental_flow_path(const std::vector<uint>& path) {
      // remove path from traffic map
      for (size_t i = 0; i < path.size() - 1; ++i) {
        visited[path[i]] = true;
        visited[path[i+1]] = true;
        uint a = path[i];
        uint b = path[i + 1];
        if(a == b) {
          vertex_flow[b] ++;
          continue;} // Skip if the same vertex
        int e_index = edge_index(a, b);
        edge_flow[e_index] ++; 
      }
    }

    void add_incremental_flow_path_from_time_index(const std::vector<uint>& path, uint start_time_index) {
      if( path.size() <= start_time_index){
        return;
      }
      for (size_t i = start_time_index; i < path.size() - 1; ++i) {
        visited[path[i]] = true;
        visited[path[i+1]] = true;
        uint a = path[i];
        uint b = path[i + 1];
        if(a == b) {
          vertex_flow[b] ++;
          continue;} // Skip if the same vertex
        int e_index = edge_index(a, b);
        edge_flow[e_index] ++; 
      }
    }


    void record_incremental_flow(){
      std::unordered_set<std::pair<int,int>, EdgePairHash> visited_edge;
      for(size_t i = 0; i < visited.size(); ++i) {
        if(visited[i]) {
          for(const auto& neighbor : G->U[i]->neighbor){
            visited_edge.insert({i, neighbor->index});
            visited_edge.insert({neighbor->index, i});
          }
        }
      }
      for(auto edge : visited_edge){
        if( edge.first == edge.second) continue;
        int edge_idx = edge_index(edge.first, edge.second);
        auto [t1, t2] = get_traffic_cost(edge.first, edge.second);
        incremental_flow[edge_idx] += (t1 + t2);
      }
    }

  void record_incremental_flow( double learning_rate){
      std::unordered_set<std::pair<int,int>, EdgePairHash> visited_edge;
      for(size_t i = 0; i < visited.size(); ++i) {
        if(visited[i]) {
          for(const auto& neighbor : G->U[i]->neighbor){
            visited_edge.insert({i, neighbor->index});
            visited_edge.insert({neighbor->index, i});
          }
        }
      }
      // std::fill(incremental_flow.begin(), incremental_flow.end(), 0);
      for(auto& edge : visited_edge){
        if( edge.first == edge.second) continue;
        int edge_idx = edge_index(edge.first, edge.second);
        auto [t1, t2] = get_traffic_cost(edge.first, edge.second);
        // incremental_flow[edge_idx] += learning_rate*(t1 + t2 );
        incremental_flow[edge_idx] +=  learning_rate*(t1 + t2 );
        // incremental_flow[edge_idx] += (t1 + t2);
        // std::cout<< " Adding regret cost on edge from "<< edge.first << " to "<< edge.second << " with cost "
        // << (t1 + t2) << std::endl;
      }
      double max_val = 0.0;
      for (double val : incremental_flow) {
          if (val > max_val) max_val = val;
      }
      if (max_val > 0) {
        for(int i = 0; i < normalized_incremental_flow.size(); ++i){
          normalized_incremental_flow[i] = incremental_flow[i] / max_val;
        }
      }
      
    }

    
    void initialize_traffic_map(const std::vector<std::vector<uint>>& agents_paths) {
      // initialize_ traffic map based on existing paths.
      for( const auto& path : agents_paths) {
        add_path(path);
      }
    }

    
    void print_incremental_flow(const std::string& flow_filename) {

      std::ofstream vout(flow_filename);
      vout << "vertex_in,vertex_out,edge_flow\n";
      for (size_t i = 0; i < G->U.size(); ++i) {
        if(G->U[i] == nullptr) continue; // skip if vertex is null
        for(const auto& neighbor : G->U[i]->neighbor) {
          uint neighbor_index = neighbor->index;
          int edge_idx = edge_index(i, neighbor_index);
          vout << i << "," << neighbor_index << "," 
              << incremental_flow[edge_idx]<<"\n";
        }
      }
      vout.close();
    }


    void print_normalized_regret_flow(const std::string& flow_filename) {
      std::ofstream vout(flow_filename);
      vout << "vertex_in,vertex_out,edge_flow\n";
      for (size_t i = 0; i < G->U.size(); ++i) {
        if(G->U[i] == nullptr) continue; // skip if vertex is null
        for(const auto& neighbor : G->U[i]->neighbor) {
          uint neighbor_index = neighbor->index;
          int edge_idx = edge_index(i, neighbor_index);
          vout << i << "," << neighbor_index << "," 
              << scaling_factor * normalized_PIBT_regret_flow[edge_idx]<<"\n";
        }
      }
      vout.close();
    }


    void print_normalized_incremental_flow(const std::string& flow_filename) {
      std::ofstream vout(flow_filename);
      vout << "vertex_in,vertex_out,edge_flow\n";
      for (size_t i = 0; i < G->U.size(); ++i) {
        if(G->U[i] == nullptr) continue; // skip if vertex is null
        for(const auto& neighbor : G->U[i]->neighbor) {
          uint neighbor_index = neighbor->index;
          int edge_idx = edge_index(i, neighbor_index);
          vout << i << "," << neighbor_index << "," 
              << scaling_factor * normalized_incremental_flow[edge_idx]<<"\n";
        }
      }
      vout.close();
    }


    void export_traffic_map_csv(const std::string& flow_filename) const {
      // Export vertex flow
      std::ofstream vout(flow_filename);
      vout << "vertex_in,vertex_out,vertex_flow,edge_flow\n";
      for (size_t i = 0; i < vertex_flow.size(); ++i) {
        if(G->U[i] == nullptr) continue; // skip if vertex is null
        for(const auto& neighbor : G->U[i]->neighbor) {
          uint neighbor_index = neighbor->index;
          int edge_idx = edge_index(i, neighbor_index);
          int edge_idx2 = edge_index(neighbor_index, i);
          vout << i << "," << neighbor_index << "," 
              << edge_flow[edge_idx] * edge_flow[edge_idx2]<< "," 
              << vertex_flow[i] << "\n";
        }
      }
      vout.close();
    }


};