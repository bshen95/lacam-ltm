/*
 * lacam-star
 */

#pragma once

#include "dist_table.hpp"
#include "graph.hpp"
#include "guidance_heuristic.hpp"
#include "instance.hpp"
#include "modified_Astar.hpp"
#include "traffic_map.hpp"
#include "utils.hpp"
// objective function
enum Objective { OBJ_NONE, OBJ_MAKESPAN, OBJ_SUM_OF_LOSS };
enum Traffic_OP {
  NONE,
  PRE_TRAFFIC,
  ONLINE_TRAFFIC,
  ONLINE_TRAFFIC_TW,
  INCRE_TRAFFIC,
  INCRE_TRAFFIC_WITH_TW,
  INCRE_PLUS_ONLINE_TRAFFIC,
  REGERT_TRAFFIC,
  TRAINNING_TRAFFIC,
  LOADING_TRAFFIC,
  CONTINUE_TRANNING,
  SIMULATION_TRAFFIC,
  PLANNING_AND_EXECUTION
};
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
  struct HNodeLessById {
    bool operator()(const HNode* a, const HNode* b) const
    {
      // define a strict weak ordering on a stable key
      return a->node_id < b->node_id;
    }
  };
  std::set<HNode*, HNodeLessById> neighbor;
  // WASTED TWO DAYS ON THIS FUKKK. without the comparator, it does not
  // guareentee to access the neigbourhe in same order, this could make the
  // results looks different even when seed is set to be the same.

  // costs
  uint g;        // g-value (might be updated)
  const uint h;  // h-value
  uint f;        // g + h (might be updated)
  uint current_make_span = 0;
  uint order_updated = 0;
  uint node_id = 0;
  // for low-level search
  std::vector<float> priorities;
  std::vector<uint> order;
  std::queue<LNode*> search_tree;

  HNode(const Config& _C, DistTable& D, HNode* _parent, const uint _g,
        const uint _h);
  ~HNode();

  void set_make_span(uint _current_make_span)
  {
    current_make_span = _current_make_span;
  }

  void set_priority_and_order(const std::vector<uint>& input_order)
  {
    // Set the order directly from the input
    order = input_order;

    // Update priorities based on the new order
    for (size_t i = 0; i < order.size(); ++i) {
      priorities[order[i]] =
          (float)(order.size() - i) /
          order.size();  // Higher priority for earlier indices
    }
  }

  void reordering_based_on_traffic(size_t N, GuidanceHeuristic& G,
                                   Traffic_OP op)
  {
    // initialize
    // for (uint i = 0; i < N; ++i) priorities[i] =
    // (double)G.get_Astar_heuristic(i, C[i]->id)/ (N) ;
    if (op == TRAINNING_TRAFFIC || op == REGERT_TRAFFIC ||
        op == LOADING_TRAFFIC || op == CONTINUE_TRANNING ||
        op == PLANNING_AND_EXECUTION) {
      for (uint i = 0; i < N; ++i) {
        priorities[i] = (double)G.get_regret_heuristic(i, C[i]->id) / (10 * N);
        // if (G.get_regret_heuristic(i, C[i]->id) == 0){
        //   priorities[i] = priorities[i] - int(priorities[i]);
        // }
      }
    } else {
      for (uint i = 0; i < N; ++i)
        priorities[i] = (double)G.get_Astar_heuristic(i, C[i]->id) / (N);
    }
    // set order
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(),
              [&](uint i, uint j) { return priorities[i] > priorities[j]; });
  }

  void reordering(size_t N, DistTable& D)
  {
    for (size_t i = 0; i < N; ++i) {
      if (D.get(i, C[i]) == 0) {
        priorities[i] = priorities[i] - (int)priorities[i];
      } else {
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
  const int planning_time;
  const int commit_steps;
  // solver utils
  const uint N;       // number of agents
  const uint V_size;  // number o vertices
  DistTable D;
  uint loop_cnt;  // auxiliary

  // used in PIBT
  std::vector<std::array<Vertex*, 5>> C_next;  // next locations, used in PIBT
  std::vector<float> tie_breakers;             // random values, used in PIBT
  Agents A;
  Agents occupied_now;   // for quick collision checking
  Agents occupied_next;  // for quick collision checking

  struct Cmp {
    bool operator()(const std::pair<double, HNode*>& a,
                    const std::pair<double, HNode*>& b) const
    {
      return a.first > b.first;
    }
  };
  std::priority_queue<std::pair<double, HNode*>,
                      std::vector<std::pair<double, HNode*>>, Cmp>
      restart_heap;

  std::vector<std::unordered_map<std::pair<int, int>, int, PairHash>>
      traffic_cache;

  // set in the constructor: true only for PLANNING_AND_EXECUTION
  bool use_global_traffic_cache = false;

  uint restart_makespan = 0;
  uint reconstruct_makespan = 20;

  HNode* global_goal_nodes;

  uint best_makespan = 0;
  uint time_bucket_size =
      10;  // Time bucket size for time-period based traffic maps
  uint current_time_bucket = 0;
  uint max_time_period = 20;  // Maximum time period for traffic maps

  uint incre_time_window_size = 50;
  uint incre_max_makespan = 0;
  uint incre_num_of_time_buckets = 0;
  uint incre_curr_time_bucket = 0;
  bool printing_tree = false;

  uint order_updated_times = 0;
  double learning_rate = 0.6;
  // double decay_rate = 0.05;

  uint curr_tw_lower_bound = 0;
  uint curr_tw_upper_bound = 0;
  uint tw_size = 50;

  uint num_of_nodes_generated = 0;
  uint solution_id = 0;

  uint node_limitation = 0;

  std::unordered_map<Config, HNode*, ConfigHasher>* GLOBAL_EXPORED;
  HNode* global_goal;
  std::vector<uint> accessed_agents;  // for quick reset of traffic map
  uint accessed_times;

  bool swap_based_on_original_distance = false;
  uint max_makespan = 0;

  std::vector<std::vector<uint>> revised_path;
  std::vector<std::vector<uint>> checked_path;
  std::vector<HNode*> node_expaned;

  HNode* restart_node;
  HNode* simulation_start_node;
  HNode* curr_goal_node;
  std::vector<HNode*> soultion_node_pool;
  std::unordered_set<HNode*> soultion_node_set;

  TrafficMap traffic_map;  // Traffic map for A* search
  std::vector<TrafficMap>
      time_period_traffic_map;  // Time-period based traffic maps for A* search
  ModifiedAstar astar_search;   // A* search for traffic path finding
  GuidanceHeuristic guidance_heuristic;  // Guidance heuristic for pathfinding

  uint traffic_pre_optimization_time = deadline->time_limit_ms / 2;
  uint restarting_times = 0;
  uint float_time_calls = 0;
  uint update_times = 0;
  uint traffic_node_added = 0;

  uint simulation_nodes_limiatation = 100;
  uint simulation_make_span_limitation = 10;

  bool is_simulation = false;
  bool load_traffic_csv = false;

  Planner(const Instance* _ins, const Deadline* _deadline, std::mt19937* _MT,
          const int _verbose = 0,
          // other parameters
          const Objective _objective = OBJ_NONE,
          const Traffic_OP _traffic = NONE, const float _restart_rate = 0.001,
          const int _planning_time = 1000, const int _commit_steps = 1);
  ~Planner();

  Solution solve_with_simulation(std::string& additional_info);

  Solution solve(std::string& additional_info);

  Solution planning_and_execution(std::string& additional_info);

  HNode* planning_next_actions(
      std::unordered_map<Config, HNode*, ConfigHasher>& EXPLORED,
      std::stack<HNode*>& OPEN, HNode** H_goal, HNode** curr_config,
      bool continue_search, double time_limits, uint curr_makespan);

  void simulate_traffic_map(HNode* H_start, uint _makespan_limit,
                            uint _node_limitation);
  void simulate_rewrite(HNode* H_from, HNode* T, HNode* H_goal,
                        std::stack<HNode*>& OPEN);

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
                                         Vertex* v_pusher_origin,
                                         Vertex* v_puller_origin);

  bool is_swap_required_gudiance_Astar_version(const uint pusher,
                                               const uint puller,
                                               Vertex* v_pusher_origin,
                                               Vertex* v_puller_origin);

  bool is_swap_required_gudiance_regret_version(const uint pusher,
                                                const uint puller,
                                                Vertex* v_pusher_origin,
                                                Vertex* v_puller_origin);

  bool is_swap_possible(Vertex* v_pusher_origin, Vertex* v_puller_origin);

  void clean_constraint(std::stack<HNode*>& OPEN);
  // traffic op

  void learning_regret_value(
      std::vector<std::array<Vertex*, 5>>& C_next_actions, const Config& C_curr,
      Config& C_next, uint current_time_step);
  void learning_traffic_cost(const Config& C_from, const Config& C_to,
                             uint current_time_step);

  void running_traffic_optimization_for_simulation(std::stack<HNode*>& OPEN,
                                                   HNode* H_goal, bool is_goal);

  void running_traffic_optimization(std::stack<HNode*>& OPEN, HNode* H_goal,
                                    bool is_goal);

  void incremental_regret_and_traffic(uint start_makespan, HNode* H_goal);
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

  void export_instance_starts_goals(const std::string& filename);

  void export_all_revised_paths(
      const std::vector<std::vector<uint>>& revised_path,
      const std::string& filename);

  void backtrack_compute_sum_of_costs(HNode* H_goal);
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
