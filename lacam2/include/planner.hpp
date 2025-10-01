/*
 * lacam-star
 */

#pragma once

#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "utils.hpp"
#include "modified_Astar.hpp"
#include "traffic_map.hpp"
#include "guidance_heuristic.hpp"
// objective function
enum Objective { OBJ_NONE, OBJ_MAKESPAN, OBJ_SUM_OF_LOSS };
enum Traffic_OP { NONE, PRE_TRAFFIC, ONLINE_TRAFFIC, ONLINE_TRAFFIC_TW, INCRE_TRAFFIC, INCRE_TRAFFIC_WITH_TW, INCRE_PLUS_ONLINE_TRAFFIC};
std::ostream& operator<<(std::ostream& os, const Objective objective);
std::ostream& operator<<(std::ostream& os, const Traffic_OP traffic);
// PIBT agent
struct Agent {
  const uint id;
  Vertex* v_now;   // current location
  Vertex* v_next;  // next location
  Agent(uint _id) : id(_id), v_now(nullptr), v_next(nullptr) {}
};
using Agents = std::vector<Agent*>;

// low-level node
struct LNode {
  std::vector<uint> who;
  Vertices where;
  const uint depth;
  LNode(LNode* parent = nullptr, uint i = 0,
        Vertex* v = nullptr);  // who and where
};

// high-level node
struct HNode {
  static uint HNODE_CNT;  // count #(high-level node)
  const Config C;

  // tree
  HNode* parent;
  std::set<HNode*> neighbor;

  // costs
  uint g;        // g-value (might be updated)
  const uint h;  // h-value
  uint f;        // g + h (might be updated)
  uint current_make_span = 0;
  uint order_updated = 0;
  // for low-level search
  std::vector<float> priorities;
  std::vector<uint> order;
  std::queue<LNode*> search_tree;

  HNode(const Config& _C, DistTable& D, HNode* _parent, const uint _g,
        const uint _h);
  ~HNode();

  void set_make_span(uint _current_make_span) {
     current_make_span = _current_make_span;
  }

  void set_priority_and_order(const std::vector<uint>& input_order) {
    // Set the order directly from the input
    order = input_order;

    // Update priorities based on the new order
    for (size_t i = 0; i < order.size(); ++i) {
        priorities[order[i]] = (float) (order.size() - i)/ order.size(); // Higher priority for earlier indices
    }
  }

  void reordering_based_on_traffic(size_t N, GuidanceHeuristic& G){
    if (parent == nullptr) {
      // initialize
      for (uint i = 0; i < N; ++i) priorities[i] = (float)G.get_Astar_heuristic(i, C[i]->id) / N;
    } 
    // set order
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(),
              [&](uint i, uint j) { return priorities[i] > priorities[j]; });
  }

  void reordering(size_t N, DistTable& D) {
    for (size_t i = 0; i < N; ++i) {
      if (D.get(i, C[i])== 0) {
        priorities[i] = priorities[i] - (int)priorities[i];
      }else{
        priorities[i] = priorities[i] + 1;
      }
    }
    // set order
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(),
              [&](uint i, uint j) { return priorities[i] > priorities[j]; });
  }

};
using HNodes = std::vector<HNode*>;

struct Planner {
  const Instance* ins;
  const Deadline* deadline;
  std::mt19937* MT;
  const int verbose;

  // hyper parameters
  const Objective objective;
  const Traffic_OP traffic_op;
  const float RESTART_RATE;  // random restart

  // solver utils
  const uint N;       // number of agents
  const uint V_size;  // number o vertices
  DistTable D;
  uint loop_cnt;      // auxiliary

  // used in PIBT
  std::vector<std::array<Vertex*, 5> > C_next;  // next locations, used in PIBT
  std::vector<float> tie_breakers;              // random values, used in PIBT
  Agents A;
  Agents occupied_now;                          // for quick collision checking
  Agents occupied_next;                         // for quick collision checking

  uint best_makespan = 0;
  uint time_bucket_size = 10; // Time bucket size for time-period based traffic maps
  uint current_time_bucket = 0;
  uint max_time_period = 20; // Maximum time period for traffic maps


  uint incre_time_window_size = 50;
  uint incre_max_makespan = 0; 
  uint incre_num_of_time_buckets = 0;
  uint incre_curr_time_bucket = 0;

  uint order_updated_times = 0;
  double learning_rate = 0.6;
  // double decay_rate = 0.05; 

  uint curr_tw_lower_bound = 0; 
  uint curr_tw_upper_bound = 0;
  uint tw_size = 50;


  uint solution_id = 0; 
  HNode* global_goal;
  std::vector<std::vector<uint>> revised_path;
  std::vector<std::vector<uint>> checked_path;


  TrafficMap traffic_map ; // Traffic map for A* search
  std::vector<TrafficMap> time_period_traffic_map; // Time-period based traffic maps for A* search
  ModifiedAstar astar_search; // A* search for traffic path finding
  GuidanceHeuristic guidance_heuristic; // Guidance heuristic for pathfinding

  Planner(const Instance* _ins, const Deadline* _deadline, std::mt19937* _MT,
          const int _verbose = 0,
          // other parameters
          const Objective _objective = OBJ_NONE,
          const Traffic_OP _traffic = NONE,
          const float _restart_rate = 0.001);
  ~Planner();


  
  Solution solve(std::string& additional_info);
  void expand_lowlevel_tree(HNode* H, LNode* L);
  void rewrite(HNode* H_from, HNode* T, HNode* H_goal,
               std::stack<HNode*>& OPEN);
  uint get_edge_cost(const Config& C1, const Config& C2);
  uint get_edge_cost(HNode* H_from, HNode* H_to);
  uint get_h_value(const Config& C);
  bool get_new_config(HNode* H, LNode* L);
  
  bool funcPIBT(Agent* ai);

  // swap operation
  Agent* swap_possible_and_required(Agent* ai);
  bool is_swap_required(const uint pusher, const uint puller,
                        Vertex* v_pusher_origin, Vertex* v_puller_origin);


  bool is_swap_required_gudiance_version(const uint pusher, const uint puller,
                        Vertex* v_pusher_origin, Vertex* v_puller_origin);
  
  bool is_swap_required_gudiance_Astar_version(const uint pusher, const uint puller,
                      Vertex* v_pusher_origin, Vertex* v_puller_origin);
                      

  bool is_swap_possible(Vertex* v_pusher_origin, Vertex* v_puller_origin);

  void clean_constraint(HNode* H_goal);
  // traffic op 

  void running_traffic_optimization(std::stack<HNode*>& OPEN, HNode* H_goal, bool is_goal);
  void incremental_increase_traffic_with_time_window(HNode* H_goal);
  void incremental_increase_traffic(HNode* H_goal);
  void incremental_increase_traffic_plus_traffic_op(HNode* H_goal);
  void traffic_optimization_tw(HNode* H_curr);
  void traffic_optimization(HNode* H_goal);
  void pre_traffic_optimization();
  void select_restart_node(std::stack<HNode*>& OPEN, HNode* H_goal);
  void propagate_order_to_neighbors(HNode* current_node);
  void learn_priority_order(HNode* H_goal);
  void get_edge_cost_per_agent(std::vector<double>& agent_cost, 
  const Config& C1, const Config& C2);
  void export_solution_from_HNode(HNode* goal, const std::string& filename);
  
  void export_all_revised_paths(const std::vector<std::vector<uint>>& revised_path, const std::string& filename);

  // utilities
  template <typename... Body>
  void solver_info(const int level, Body&&... body)
  {
    if (verbose < level) return;
    std::cout << "elapsed:" << std::setw(6) << elapsed_ms(deadline) << "ms"
              << "  loop_cnt:" << std::setw(8) << loop_cnt
              << "  node_cnt:" << std::setw(8) << HNode::HNODE_CNT << "\t";
    info(level, verbose, (body)...);
  }
};
