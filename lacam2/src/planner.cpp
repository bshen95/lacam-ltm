#include "../include/planner.hpp"

LNode::LNode(LNode* parent, uint i, Vertex* v)
    : who(), where(), depth(parent == nullptr ? 0 : parent->depth + 1)
{
  if (parent != nullptr) {
    who = parent->who;
    who.push_back(i);
    where = parent->where;
    where.push_back(v);
  }
}

uint HNode::HNODE_CNT = 0;

// for high-level
HNode::HNode(const Config& _C, DistTable& D, HNode* _parent, const uint _g,
             const uint _h)
    : C(_C),
      parent(_parent),
      neighbor(),
      g(_g),
      h(_h),
      f(g + h),
      priorities(C.size()),
      order(C.size(), 0),
      search_tree(std::queue<LNode*>())
{
  ++HNODE_CNT;

  search_tree.push(new LNode());
  const auto N = C.size();

  // update neighbor
  if (parent != nullptr) parent->neighbor.insert(this);

  // set priorities
  if (parent == nullptr) {
    // initialize
    for (uint i = 0; i < N; ++i) priorities[i] = (float)D.get(i, C[i]) / N;
  } else {
    // dynamic priorities, akin to PIBT
    for (size_t i = 0; i < N; ++i) {
      if (D.get(i, C[i]) != 0) {
        priorities[i] = parent->priorities[i] + 1;
      } else {
        priorities[i] = parent->priorities[i] - (int)parent->priorities[i];
      }
    }
  }

  // set order
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
            [&](uint i, uint j) { return priorities[i] > priorities[j]; });
}

HNode::~HNode()
{
  while (!search_tree.empty()) {
    delete search_tree.front();
    search_tree.pop();
  }
}

Planner::Planner(const Instance* _ins, const Deadline* _deadline,
                 std::mt19937* _MT, const int _verbose,
                 const Objective _objective, const Traffic_OP _traffic,
                 const float _restart_rate)
    : ins(_ins),
      deadline(_deadline),
      MT(_MT),
      verbose(_verbose),
      objective(_objective),
      traffic_op(_traffic),
      RESTART_RATE(_restart_rate),
      N(ins->N),
      V_size(ins->G.size()),
      D(DistTable(ins)),
      loop_cnt(0),
      C_next(N),
      tie_breakers(V_size, 0),
      A(N, nullptr),
      occupied_now(V_size, nullptr),
      occupied_next(V_size, nullptr),
      traffic_map(&ins->G, ins->N),
      time_period_traffic_map(incre_num_of_time_buckets,
                              TrafficMap(&ins->G, ins->N)),
      astar_search(&traffic_map, &ins->G),
      guidance_heuristic(&ins->G, &traffic_map, N)
{
}

Planner::~Planner() {}

void Planner::incremental_regret_and_traffic(HNode* H_goal)
{
  guidance_heuristic.initialized = true;
  // int solution_makespan =  H_goal->current_make_span +1 ;
  // HNode* current = H_goal;
  // std::vector<std::vector<uint>> solution_nodes =
  // std::vector<std::vector<uint>>(N); for (uint i = 0; i < N; ++i) {
  //   solution_nodes[i].push_back(H_goal->C[i]->index);
  // }
  // while (current != restart_node->parent) {
  //   for(uint i = 0; i < N; ++i) {
  //     if(solution_nodes[i].size() == 1
  //       && current->C[i]->index == H_goal->C[i]->index){
  //         continue; // skip if already at goal
  //     }
  //     solution_nodes[i].push_back(current->C[i]->index);
  //   }
  //   current = current->parent;
  // }
  // for (uint i = 0; i < N; ++i) {
  //   std::reverse(solution_nodes[i].begin(), solution_nodes[i].end());
  // }
  // for(auto solution : solution_nodes){
  //   traffic_map.increase_regret_based_on_path(solution);
  // }
  traffic_map.record_regret_cost(learning_rate);
  traffic_map.reset_regret_cost();
  guidance_heuristic.reset(ins);
  guidance_heuristic.set_traffic_map(&traffic_map);
}
void Planner::incremental_increase_traffic(HNode* H_goal)
{
  guidance_heuristic.initialized = true;
  traffic_map.reset();
  int solution_makespan = H_goal->current_make_span + 1;
  HNode* current = H_goal;
  std::vector<std::vector<uint>> solution_nodes =
      std::vector<std::vector<uint>>(N);
  for (uint i = 0; i < N; ++i) {
    solution_nodes[i].push_back(H_goal->C[i]->index);
  }
  while (current != restart_node->parent) {
    for (uint i = 0; i < N; ++i) {
      if (solution_nodes[i].size() == 1 &&
          current->C[i]->index == H_goal->C[i]->index) {
        continue;  // skip if already at goal
      }
      solution_nodes[i].push_back(current->C[i]->index);
    }
    current = current->parent;
  }
  for (uint i = 0; i < N; ++i) {
    std::reverse(solution_nodes[i].begin(), solution_nodes[i].end());
  }
  for (auto solution : solution_nodes) {
    traffic_map.add_incremental_flow_path(solution);
  }
  traffic_map.record_incremental_flow(learning_rate);
  guidance_heuristic.reset(ins);
  guidance_heuristic.set_traffic_map(&traffic_map);
}

void Planner::incremental_increase_traffic_with_time_window(HNode* H_goal)
{
  guidance_heuristic.initialized = true;
  int solution_makespan = H_goal->current_make_span + 1;
  HNode* current = H_goal;
  std::vector<std::vector<uint>> solution_nodes =
      std::vector<std::vector<uint>>(N);
  for (uint i = 0; i < N; ++i) {
    solution_nodes[i].push_back(H_goal->C[i]->index);
  }
  while (current != restart_node->parent) {
    for (uint i = 0; i < N; ++i) {
      if (solution_nodes[i].size() == 1 &&
          current->C[i]->index == H_goal->C[i]->index) {
        continue;  // skip if already at goal
      }
      solution_nodes[i].push_back(current->C[i]->index);
    }
    current = current->parent;
  }
  for (uint i = 0; i < N; ++i) {
    std::reverse(solution_nodes[i].begin(), solution_nodes[i].end());
  }

  // while (current != restart_node) {
  //   current = current->parent;
  //   for(uint i = 0; i < N; ++i) {
  //     solution_nodes[i].push_back(current->C[i]->index);
  //   }
  // }

  // for (uint i = 0; i < N; ++i) {
  //   if(solution_nodes[i].back() != ins->starts[i]->index){
  //     std::cout<<"Error: solution path does not reach the restart
  //     node"<<std::endl;
  //   }
  //   std::reverse(solution_nodes[i].begin(), solution_nodes[i].end());
  // }

  if (incre_max_makespan == 0) {
    incre_max_makespan = solution_makespan * 2;
    incre_num_of_time_buckets = static_cast<int>(
        std::ceil(static_cast<double>(incre_max_makespan) /
                  static_cast<double>(incre_time_window_size)));
    if (incre_num_of_time_buckets < 1) {
      std::cout << "Error: incre_num_of_time_buckets < 1" << std::endl;
    }
    time_period_traffic_map.resize(incre_num_of_time_buckets,
                                   TrafficMap(&ins->G, ins->N));
  }
  for (uint t = 0; t < incre_num_of_time_buckets; t++) {
    time_period_traffic_map[t].reset();
    for (auto& solution : solution_nodes) {
      uint start_index = t * incre_time_window_size;
      time_period_traffic_map[t].add_incremental_flow_path_from_time_index(
          solution, start_index);
    }
    time_period_traffic_map[t].record_incremental_flow(learning_rate);
  }

  incre_curr_time_bucket = 0;
  // traffic_map.print_incremental_flow("incremental_flow.csv");
  guidance_heuristic.reset(ins);
  guidance_heuristic.set_traffic_map(
      &time_period_traffic_map[incre_curr_time_bucket]);

  // uint incre_time_window_size = 40;
  //   uint incre_max_makespan = 0;
  //   uint incre_num_of_time_buckets = 0;
  //   uint incre_curr_time_bucket = 0;

  // if(best_makespan == 0){
  //   best_makespan = solution_makespan * 2;
  //   // max_time_period = best_makespan/time_bucket_size;
  //   // max_time_period =
  //   static_cast<int>(std::ceil(static_cast<double>(best_makespan) /
  //   time_bucket_size)); max_time_period =
  //   static_cast<int>(std::ceil(static_cast<double>(best_makespan) /
  //   time_bucket_size)); if(max_time_period < 1){
  //     max_time_period = 1;
  //   }
  //   // std::cout<<"Initial max time period: " << best_makespan << std::endl;
  // }
  // for(uint t = 0; t < time_bucket_size; t++){
  //   time_period_traffic_map[t].reset();
  // }

  // for(auto solution : solution_nodes){
  //   for(uint t = 0; t < time_bucket_size; t++){
  //     uint start_index = t * max_time_period;
  //     time_period_traffic_map[t].add_incremental_flow_path_from_time_index(solution,
  //     start_index);
  //   }
  // }
  // for(uint t = 0; t < time_bucket_size; t++){
  //   time_period_traffic_map[t].record_incremental_flow(learning_rate);
  // }

  // current_time_bucket = 0;
  // // traffic_map.print_incremental_flow("incremental_flow.csv");
  // guidance_heuristic.reset(ins);
  // guidance_heuristic.set_traffic_map(&time_period_traffic_map[current_time_bucket]);
  // std::cout<<"finishing increasing traffic map" << std::endl;
}

void Planner::incremental_increase_traffic_plus_traffic_op(HNode* H_goal)
{
  guidance_heuristic.initialized = true;
  traffic_map.reset();
  int solution_makespan = H_goal->current_make_span + 1;
  HNode* current = H_goal;
  std::vector<std::vector<uint>> solution_nodes =
      std::vector<std::vector<uint>>(N);
  for (uint i = 0; i < N; ++i) {
    solution_nodes[i].push_back(H_goal->C[i]->index);
  }
  while (current->parent != nullptr) {
    for (uint i = 0; i < N; ++i) {
      solution_nodes[i].push_back(current->C[i]->index);
    }
    current = current->parent;
  }
  for (uint i = 0; i < N; ++i) {
    solution_nodes[i].push_back(ins->starts[i]->index);
    std::reverse(solution_nodes[i].begin(), solution_nodes[i].end());
  }
  for (auto solution : solution_nodes) {
    traffic_map.add_incremental_flow_path(solution);
  }
  traffic_map.record_incremental_flow(learning_rate);

  // //considering previous traffic to optimize the flow;
  // if(H_goal->h == 0){
  //   // at goal
  //   HNode* current = H_goal;
  //   std::vector<std::vector<uint>> solution_nodes =
  //   std::vector<std::vector<uint>>(N); for (uint i = 0; i < N; ++i) {
  //     solution_nodes[i].push_back(H_goal->C[i]->index);
  //   }
  //   while (current->parent != nullptr) {
  //     for(uint i = 0; i < N; ++i) {
  //       solution_nodes[i].push_back(current->C[i]->index);
  //     }
  //     current = current->parent;
  //   }
  //   for (uint i = 0; i < N; ++i) {
  //     solution_nodes[i].push_back(ins->starts[i]->index);
  //     std::reverse(solution_nodes[i].begin(), solution_nodes[i].end());
  //   }
  //   revised_path = solution_nodes;
  // }

  traffic_map.reset();
  revised_path.clear();
  for (int i = 0; i < N; i++) {
    auto path = astar_search.compute_traffic_path_index_consider_past_traffic(
        ins->starts[i]->index, ins->goals[i]->index);
    traffic_map.add_path(path);
    revised_path.push_back(path);
  }
  // traffic_map.initialize_traffic_map(revised_path);

  std::vector<uint> ordering = std::vector<uint>(N);
  std::iota(ordering.begin(), ordering.end(), 0);
  // std::shuffle(ordering.begin(),ordering.end(), *MT);
  int times = 10;
  while (times > 0) {
    // std::shuffle(ordering.begin(),ordering.end(), *MT);
    for (size_t i = 0; i < ordering.size(); ++i) {
      int agent_id = ordering[i];
      traffic_map.remove_path(revised_path[agent_id]);
      auto path = astar_search.compute_traffic_path_index_consider_past_traffic(
          ins->starts[agent_id]->index, ins->goals[agent_id]->index);
      if (path.empty()) {
        std::cout << "No path found for agent " << agent_id << std::endl;
        continue;  // No path found, skip this agent
      }
      traffic_map.add_path(path);
      revised_path[agent_id] = path;
      // update the edge weights in the traffic map
    }
    times--;
  }
  guidance_heuristic.set_gudiance_path(revised_path);
}

void Planner::pre_traffic_optimization()
{
  // std::cout<<" start optimizing "<<std::endl;
  traffic_map.reset();
  revised_path.clear();
  for (int i = 0; i < N; i++) {
    auto path = astar_search.compute_traffic_path_index(ins->starts[i]->index,
                                                        ins->goals[i]->index);
    traffic_map.add_path(path);
    revised_path.push_back(path);
    if (is_expired(deadline)) {
      // fail to optimize traffic.
      return;
    }
    if (is_expired_time(deadline, traffic_pre_optimization_time)) {
      // std::cout << "unfinished" << std::endl;
      guidance_heuristic.set_gudiance_path(revised_path);
      return;
    }
  }
  // std::cout<<"finish path computataion optimizing "<<std::endl;
  // traffic_map.initialize_traffic_map(revised_path);
  std::vector<uint> ordering = std::vector<uint>(N);
  std::iota(ordering.begin(), ordering.end(), 0);
  // std::shuffle(ordering.begin(),ordering.end(), *MT);
  bool stop = false;
  while (!stop) {
    for (size_t i = 0; i < ordering.size(); ++i) {
      int agent_id = ordering[i];
      traffic_map.remove_path(revised_path[agent_id]);
      auto path = astar_search.compute_traffic_path_index(
          ins->starts[agent_id]->index, ins->goals[agent_id]->index);
      if (path.empty()) {
        // std::cout << "No path found for agent " << agent_id << std::endl;
        continue;  // No path found, skip this agent
      }
      traffic_map.add_path(path);
      revised_path[agent_id] = path;
      // update the edge weights in the traffic map
      if (is_expired(deadline) ||
          is_expired_time(deadline, traffic_pre_optimization_time)) {
        stop = true;
        break;
      }
    }
  }
  guidance_heuristic.set_gudiance_path(revised_path);
}

void Planner::export_all_revised_paths(
    const std::vector<std::vector<uint>>& revised_path,
    const std::string& filename)
{
  std::ofstream fout(filename);
  fout << "agent_id,time_step,vertex_index\n";
  for (size_t agent = 0; agent < revised_path.size(); ++agent) {
    for (size_t t = 0; t < revised_path[agent].size(); ++t) {
      fout << agent << "," << t << "," << revised_path[agent][t] << "\n";
    }
  }
  fout.close();
}

void Planner::traffic_optimization_tw(HNode* H_curr)
{
  traffic_map.reset();
  revised_path.clear();
  for (int i = 0; i < N; i++) {
    auto path = astar_search.compute_traffic_path_index(H_curr->C[i]->index,
                                                        ins->goals[i]->index);
    revised_path.push_back(path);
    traffic_map.add_path(path);
  }
  std::vector<uint> ordering = std::vector<uint>(N);
  std::iota(ordering.begin(), ordering.end(), 0);
  // ordering = H_curr->order;
  // std::shuffle(ordering.begin(),ordering.end(), *MT);
  int times = 10;
  while (times > 0) {
    for (size_t i = 0; i < ordering.size(); ++i) {
      int agent_id = ordering[i];
      traffic_map.remove_path(revised_path[agent_id]);
      auto path = astar_search.compute_traffic_path_index(
          H_curr->C[agent_id]->index, ins->goals[agent_id]->index);
      if (path.empty()) {
        std::cout << "No path found for agent " << agent_id << std::endl;
        continue;  // No path found, skip this agent
      }
      traffic_map.add_path(path);
      revised_path[agent_id] = path;
      // update the edge weights in the traffic map
    }
    times--;
  }
  // int i = 0;
  // for(auto p : revised_path){
  //   std::cout << i << "," << p.size() << std::endl;
  //   i++;
  // }
  // export_all_revised_paths(revised_path,"revised_path.csv");
  guidance_heuristic.set_gudiance_path(revised_path);
}

void Planner::traffic_optimization(HNode* H_goal)
{
  if (H_goal->h == 0) {
    // at goal
    HNode* current = H_goal;
    std::vector<std::vector<uint>> solution_nodes =
        std::vector<std::vector<uint>>(N);
    for (uint i = 0; i < N; ++i) {
      solution_nodes[i].push_back(H_goal->C[i]->index);
    }
    while (current->parent != nullptr) {
      for (uint i = 0; i < N; ++i) {
        solution_nodes[i].push_back(current->C[i]->index);
      }
      current = current->parent;
    }
    for (uint i = 0; i < N; ++i) {
      solution_nodes[i].push_back(ins->starts[i]->index);
      std::reverse(solution_nodes[i].begin(), solution_nodes[i].end());
    }
    traffic_map.reset();
    traffic_map.initialize_traffic_map(solution_nodes);
    revised_path = solution_nodes;
  }

  // traffic_map.reset();
  // revised_path.clear();
  // for( int i = 0 ; i < N ; i++){
  //   auto path =
  //   astar_search.compute_traffic_path_index(ins->starts[i]->index,
  //   ins->goals[i]->index); revised_path.push_back(path);
  //   traffic_map.add_path(path);
  // }

  std::vector<uint> ordering = std::vector<uint>(N);
  std::iota(ordering.begin(), ordering.end(), 0);
  // std::shuffle(ordering.begin(),ordering.end(), *MT);
  int times = 10;
  while (times > 0) {
    // std::shuffle(ordering.begin(),ordering.end(), *MT);
    for (size_t i = 0; i < ordering.size(); ++i) {
      int agent_id = ordering[i];
      traffic_map.remove_path(revised_path[agent_id]);
      auto path = astar_search.compute_traffic_path_index(
          ins->starts[agent_id]->index, ins->goals[agent_id]->index);
      if (path.empty()) {
        std::cout << "No path found for agent " << agent_id << std::endl;
        continue;  // No path found, skip this agent
      }
      traffic_map.add_path(path);
      revised_path[agent_id] = path;
      // update the edge weights in the traffic map
      if (is_expired(deadline)) {
        // run optimization for 10 sec;
        return;
      }
    }
    times--;
  }
  guidance_heuristic.set_gudiance_path(revised_path);
}

void Planner::clean_constraint(HNode* H_goal)
{
  HNode* current = H_goal;
  while (current->parent != nullptr) {
    while (!current->parent->search_tree.empty()) {
      delete current->parent->search_tree.front();
      current->parent->search_tree.pop();
    }
    current->parent->search_tree.push(new LNode());
    // current->parent->order_updated = order_updated_times;
    // // TODO: propagate the order to neighbourhood.
    // propagate_order_to_neighbors(current->parent);
    current = current->parent;
  }
}

void Planner::select_restart_node(std::stack<HNode*>& OPEN, HNode* H_goal)
{
  // // std::cout<< "Selecting restart node..." << std::endl;
  // uint curr_make_span = curr_goal_node->current_make_span;
  // // std::cout<< "Current makespan: " << curr_make_span << std::endl;
  // uint backtrack =  get_random_int(MT, 0, curr_make_span);
  // // uint backtrack = 150;
  // // std::cout << "Restart from backtrack: " << backtrack << std::endl;
  // HNode* current = H_goal;
  // while (current->parent != nullptr ) {
  //   // backtrack--;
  //   current = current->parent;
  // }
  // while (!current->search_tree.empty()) {
  //   delete current->search_tree.front();
  //   current->search_tree.pop();
  // }
  // current->search_tree.push(new LNode());
  // current->reordering_based_on_traffic(N,guidance_heuristic,traffic_op);

  // uint backtrack =  get_random_int(MT, 0, soultion_node_pool.size()-1);
  OPEN = std::stack<HNode*>();
  while (!restart_heap.empty()) {
    auto next_pair = restart_heap.top();
    restart_heap.pop();
    HNode* next_node = next_pair.second;
    if (next_node->f > curr_goal_node->f) {
      continue;
    } else {
      // std::cout<<"pop "<< next_pair.first << "  " <<
      // next_pair.second->current_make_span<< std::endl;
      OPEN.push(next_node);
      while (!next_node->search_tree.empty()) {
        delete next_node->search_tree.front();
        next_node->search_tree.pop();
      }
      next_node->search_tree.push(new LNode());
      next_node->reordering_based_on_traffic(N, guidance_heuristic, traffic_op);
      // restart_heap.size();
      restart_heap.push(
          std::make_pair(next_pair.first + restart_heap.size(), next_node));
      return;
    }
  }
}

void Planner::propagate_order_to_neighbors(HNode* current_node)
{
  // Iterate through all neighbors of the current node
  for (auto neighbor : current_node->neighbor) {
    // Update the neighbor's order based on the current node's order
    if (neighbor->order_updated != order_updated_times) {
      neighbor->set_priority_and_order(current_node->order);
      neighbor->reordering(N, D);
      while (!neighbor->search_tree.empty()) {
        delete neighbor->search_tree.front();
        neighbor->search_tree.pop();
      }
      neighbor->search_tree.push(new LNode());

      neighbor->order_updated = order_updated_times;
      propagate_order_to_neighbors(neighbor);
    }
  }
}

void Planner::get_edge_cost_per_agent(std::vector<double>& agent_cost,
                                      const Config& C1, const Config& C2)
{
  if (objective == OBJ_SUM_OF_LOSS) {
    for (uint i = 0; i < N; ++i) {
      if (C1[i] != ins->goals[i] || C2[i] != ins->goals[i]) {
        agent_cost[i] += 1;
      }
    }
  } else {
    // default: makespan
    for (uint i = 0; i < N; ++i) {
      agent_cost[i] += 1;
    }
  }
}
void Planner::learn_priority_order(HNode* input_H_goal)
{
  std::vector<double> agent_cost(N, 0);
  std::vector<double> agent_ratio(N, 0);
  order_updated_times++;
  HNode* current = input_H_goal;
  current->order_updated = order_updated_times;
  current = input_H_goal;
  while (current->parent != nullptr) {
    get_edge_cost_per_agent(agent_cost, current->C, current->parent->C);

    for (uint i = 0; i < N; ++i) {
      if (D.get(i, current->parent->C[i]) == 0) {
        // a large number
        agent_ratio[i] = 10000;
      } else {
        agent_ratio[i] = agent_cost[i] / (D.get(i, current->parent->C[i]) -
                                          D.get(i, input_H_goal->C[i]));
      }
    }

    // std::uniform_real_distribution<double> noise_dist(-0.1, 0.1); // Adjust
    // range as needed for (size_t i = 0; i < agent_ratio.size(); ++i) {
    //     agent_ratio[i] = agent_ratio[i] * (1 + noise_dist(*MT));
    // }
    std::sort(
        current->parent->order.begin(), current->parent->order.end(),
        [&](uint i, uint j) { return (agent_ratio[i]) > (agent_ratio[j]); });
    current->parent->set_priority_and_order(current->parent->order);
    current->parent->reordering(N, D);
    while (!current->parent->search_tree.empty()) {
      delete current->parent->search_tree.front();
      current->parent->search_tree.pop();
    }
    current->parent->search_tree.push(new LNode());
    current->parent->order_updated = order_updated_times;
    // TODO: propagate the order to neighbourhood.
    propagate_order_to_neighbors(current->parent);
    current = current->parent;
    // makespan --;
  }
}

void Planner::export_solution_from_HNode(HNode* goal,
                                         const std::string& filename)
{
  // Backtrack from goal to root, collecting configurations
  std::vector<Config> solution;
  HNode* current = goal;
  while (current != nullptr) {
    solution.push_back(current->C);
    current = current->parent;
  }
  std::reverse(solution.begin(), solution.end());

  // std::ofstream fout2("length" + filename);
  // fout2 << "agent_id,length\n";
  // for(size_t agent = 0; agent < ins->N; ++agent) {
  //     int length = 0;
  //     for (size_t t = 0; t < solution.size(); ++t) {
  //         if (solution[t][agent] != ins->goals[agent]) {
  //             length++;
  //         }
  //     }
  //     fout2 << agent << "," << length << "\n";
  // }
  // fout2.close();

  // Export as CSV: agent_id, time_step, vertex_index
  std::ofstream fout(filename);
  fout << "agent_id,time_step,vertex_index\n";
  for (size_t t = 0; t < solution.size(); ++t) {
    for (size_t agent = 0; agent < solution[t].size(); ++agent) {
      fout << agent << "," << t << "," << solution[t][agent]->index << "\n";
    }
  }
  fout.close();
}

void Planner::running_traffic_optimization(std::stack<HNode*>& OPEN,
                                           HNode* H_goal, bool is_goal)
{
  // if(traffic_op == INCRE_TRAFFIC){
  // std::cout<< " increasing "<< std::endl;

  // if(is_goal){
  //   learn_priority_order(H_goal);
  // }
  // std::cout<< "restarting........"<<std::endl;
  // std::cout<< "f:"<<H_goal->f << "g:"<<H_goal->g << "h:"<<
  // H_goal->h<<std::endl;

  if (is_goal) {
    // export_solution_from_HNode(H_goal,"solution_to_" +
    // std::to_string(solution_id)+".csv");
    // traffic_map.print_normalized_regret_flow("incremental_flow_to_" +
    // std::to_string(solution_id)+".csv");
    solution_id++;
  }
  // if(is_goal){
  //     learning_rate = 0.6;
  // }else{
  //     learning_rate = 1.2;
  // }
  // std::cout<<" Restarting: " << restarting_times << " "<< std::endl;
  // std::cout<<" Restarting: " << restarting_times << " "<< H_goal->f <<" " <<
  // H_goal->h << " " << H_goal->g << std::endl; restarting_times ++;

  // if(is_goal)
  // for (auto n: node_expaned){
  //   while (!n->search_tree.empty()) {
  //       delete n->search_tree.front();
  //       n->search_tree.pop();
  //     }
  //     n->search_tree.push(new LNode());
  //     // n->reordering_based_on_traffic(N,guidance_heuristic,traffic_op);
  // }
  // node_expaned.clear();

  if (is_goal) {
    curr_goal_node = H_goal;
    HNode* current = H_goal;
    soultion_node_pool.clear();
    // restart_heap = std::priority_queue<std::pair<double,HNode*>,
    // std::vector<std::pair<double,HNode*>>, Cmp>();
    while (current->parent != nullptr) {
      soultion_node_pool.push_back(current);
      double distance_to_goal = H_goal->g - current->g;
      double shortest_distance_to_goal = 0;
      for (int agent_id = 0; agent_id < N; agent_id++) {
        shortest_distance_to_goal += D.get(agent_id, current->C[agent_id]);
      }
      // restart_heap.push(std::make_pair(distance_to_goal/shortest_distance_to_goal,current));
      // double random = get_random_float(MT, 0.0, 0.5);
      // restart_heap.push(std::make_pair(
      //     // current->current_make_span / curr_goal_node->current_make_span,
      //     distance_to_goal / shortest_distance_to_goal, current));
      restart_heap.push(std::make_pair(
          current->current_make_span / curr_goal_node->current_make_span,
          current));
      current = current->parent;
    }
    // snapshot the flow map when updating solution.
    if (traffic_op == TRAINNING_TRAFFIC || traffic_op == CONTINUE_TRANNING) {
      traffic_map.snapshot_flow_map();
    }
    // export_solution_from_HNode(H_goal,"solution_to_" +
    // std::to_string(solution_id)+".csv");
    // traffic_map.print_normalized_regret_flow("incremental_flow_to_" +
    // std::to_string(solution_id)+".csv"); solution_id ++;
  }

  learning_rate = 1.0;
  // learn_priority_order(H_goal);
  if (traffic_op == ONLINE_TRAFFIC) {
    traffic_optimization(H_goal);
  } else if (traffic_op == INCRE_TRAFFIC) {
    incremental_increase_traffic(H_goal);
  } else if (traffic_op == INCRE_TRAFFIC_WITH_TW) {
    incremental_increase_traffic_with_time_window(H_goal);
  } else if (traffic_op == INCRE_PLUS_ONLINE_TRAFFIC) {
    incremental_increase_traffic_plus_traffic_op(H_goal);
  } else if (traffic_op == REGERT_TRAFFIC || traffic_op == TRAINNING_TRAFFIC ||
             traffic_op == LOADING_TRAFFIC || traffic_op == CONTINUE_TRANNING) {
    // incremental_increase_traffic(H_goal);
    incremental_regret_and_traffic(H_goal);
  }
  // clean_constraint(H_goal);
  select_restart_node(OPEN, H_goal);

  node_limitation = 0;

  // learn_priority_order(H_goal);
}

Solution Planner::solve(std::string& additional_info)
{
  // std::cout<<objective<<","<<traffic_op<<std::endl;
  solver_info(1, "start search");
  restart_heap =
      std::priority_queue<std::pair<double, HNode*>,
                          std::vector<std::pair<double, HNode*>>, Cmp>();
  accessed_agents.resize(N, 0);
  accessed_times = 0;
  soultion_node_pool = std::vector<HNode*>();
  num_of_nodes_generated = 0;
  checked_path.clear();
  if (traffic_op != NONE) {
    order_updated_times = 0;
    incre_max_makespan = 0;
    incre_num_of_time_buckets = 0;
    incre_curr_time_bucket = 0;
    guidance_heuristic.initialized = false;
    if (traffic_op != PRE_TRAFFIC && traffic_op != ONLINE_TRAFFIC) {
      guidance_heuristic.setup(ins);
    }
    if (traffic_op == PRE_TRAFFIC) {
      pre_traffic_optimization();
    }
  }
  // setup agents
  for (auto i = 0; i < N; ++i) A[i] = new Agent(i);

  // setup search
  auto OPEN = std::stack<HNode*>();
  auto EXPLORED = std::unordered_map<Config, HNode*, ConfigHasher>();
  // insert initial node, 'H': high-level node
  auto H_init = new HNode(ins->starts, D, nullptr, 0, get_h_value(ins->starts));
  H_init->set_make_span(0);
  num_of_nodes_generated = 1;
  H_init->node_id = num_of_nodes_generated;
  num_of_nodes_generated++;
  OPEN.push(H_init);
  EXPLORED[H_init->C] = H_init;

  GLOBAL_EXPORED = &EXPLORED;
  restart_node = H_init;
  std::vector<Config> solution;
  auto C_new = Config(N, nullptr);  // for new configuration
  HNode* H_goal = nullptr;          // to store goal node
  global_goal = H_goal;

  HNode* H_last = nullptr;

  // DFS
  if (printing_tree) {
    std::cout << "version: 1.4.0\n";
    std::cout << "events:\n";
  }

  if (printing_tree) {
    std::cout << " - type: generating" << std::endl;
    std::cout << "   id: " << H_init->node_id << std::endl;
    std::cout << "   pId: "
              << (H_init->parent == nullptr
                      ? "0"
                      : std::to_string(H_init->parent->node_id))
              << std::endl;
    std::cout << "   f_value: " << H_init->f << std::endl;
  }
  if (traffic_op == LOADING_TRAFFIC || traffic_op == CONTINUE_TRANNING) {
    if (load_traffic_csv) {
      guidance_heuristic.initialized = true;
      H_init->reordering_based_on_traffic(N, guidance_heuristic, traffic_op);
    }
    // auto expanded = D.get_expended_vertex();
    // int cout = 0;
    // for( int v = 0 ; v < expanded.size(); v++){
    //   if(expanded[v] == true){
    //     cout ++;
    //   }
    // }
    // // std::cout<< "Loaded traffic map, expanded nodes: " << cout << "/" <<
    // expanded.size() << std::endl;
    // traffic_map.remove_visited_node_and_renormalized(D.get_expended_vertex());
  }

  // double a = guidance_heuristic.get_regret_heuristic(42, 581);
  // double b = guidance_heuristic.get_regret_heuristic(42, ins->goals[42]->id);
  // double c = guidance_heuristic.get_regret_heuristic(42, 580);
  // double d = guidance_heuristic.get_regret_heuristic(42, 582);
  // double e = guidance_heuristic.get_regret_heuristic(42, 579);
  // double f = guidance_heuristic.get_regret_heuristic(42, 578);
  // double g = guidance_heuristic.get_regret_heuristic(42, 607);
  //   double h = guidance_heuristic.get_regret_heuristic(42, 606);
  // double aaaa = 0;
  while (!OPEN.empty() && !is_expired(deadline)) {
    // if(OPEN.size() == 1){
    //   for(auto nn: EXPLORED){
    //     nn.second->reordering_based_on_traffic(N,guidance_heuristic);
    //   }
    // }
    if (traffic_op == PRE_TRAFFIC && !guidance_heuristic.initialized) {
      break;
    }
    loop_cnt += 1;

    // do not pop here!
    auto H = OPEN.top();  // high-level node
    H_last = H;
    // low-level search end
    if (H->search_tree.empty()) {
      OPEN.pop();
      continue;
    }

    // check lower bounds
    if (H_goal != nullptr && H->f >= H_goal->f) {
      OPEN.pop();
      if (traffic_op != NONE && traffic_op != PRE_TRAFFIC) {
        running_traffic_optimization(OPEN, H, false);
      }
      continue;
    }

    if (traffic_op == REGERT_TRAFFIC || traffic_op == TRAINNING_TRAFFIC ||
        traffic_op == LOADING_TRAFFIC || traffic_op == CONTINUE_TRANNING) {
      if (H_goal != nullptr) {
        node_limitation++;
        // std::cout<<" hahfhsdfhshfdsahfdshf"<<std::endl;
        if (node_limitation >
            (H->current_make_span - restart_node->current_make_span) * 10) {
          if (traffic_op != NONE && traffic_op != PRE_TRAFFIC) {
            running_traffic_optimization(OPEN, H, false);
          }
        }
      }
    }
    // check goal condition
    if (H_goal == nullptr && is_same_config(H->C, ins->goals)) {
      H_goal = H;
      solver_info(1, "found solution, cost: ", H->g);
      if (printing_tree) {
        std::cout << " - type: expanding" << std::endl;
        std::cout << "   id: " << H->node_id << std::endl;
        std::cout << "   pId: "
                  << (H->parent == nullptr ? "0"
                                           : std::to_string(H->parent->node_id))
                  << std::endl;
        std::cout << "   f_value: " << H->f << std::endl;
      }
      // export_solution_from_HNode(H_goal,"solution_same.csv");
      // traffic_map.print_normalized_regret_flow("regret_flow.csv");
      if (objective == OBJ_NONE) break;
      if (traffic_op != NONE && traffic_op != PRE_TRAFFIC) {
        running_traffic_optimization(OPEN, H_goal, true);
      }
      continue;
    }
    if (traffic_op == ONLINE_TRAFFIC_TW) {
      if (H->current_make_span == 0) {
        // std::cout<<"Running Traffic:" << H->current_make_span
        // <<","<<std::endl;
        traffic_optimization_tw(H);
        curr_tw_lower_bound = 0;
        curr_tw_upper_bound = curr_tw_lower_bound + tw_size;
        // std::cout<< "curr_tw_lower_bound" << curr_tw_lower_bound  <<
        // "curr_tw_upper_bound" << curr_tw_upper_bound << std::endl;

      } else if (H->current_make_span < curr_tw_lower_bound ||
                 H->current_make_span > curr_tw_upper_bound) {
        // std::cout<<"Running Traffic" << H->current_make_span
        // <<","<<std::endl;
        traffic_optimization_tw(H);
        curr_tw_lower_bound = (int)(H->current_make_span / tw_size) * tw_size;
        curr_tw_upper_bound = curr_tw_lower_bound + tw_size;
        // std::cout<< "curr_tw_lower_bound" << curr_tw_lower_bound  <<
        // "curr_tw_upper_bound" << curr_tw_upper_bound << std::endl;
      }
    }
    if (traffic_op == INCRE_TRAFFIC_WITH_TW) {
      if (guidance_heuristic.initialized != false) {
        uint start_index = static_cast<int>(
            std::floor(static_cast<double>(H->current_make_span) /
                       static_cast<double>(incre_time_window_size)));
        // std::cout<< "Current time step: " << H->current_make_span << ", time
        // bucket: " << start_index << std::endl;
        if (start_index != incre_curr_time_bucket) {
          // set traffic based on time period;
          incre_curr_time_bucket = start_index;
          guidance_heuristic.set_traffic_map(
              &time_period_traffic_map[incre_curr_time_bucket]);
          guidance_heuristic.reset(ins);
          // std::cout<< "Updating traffic map to time bucket: " <<
          // current_time_bucket << std::endl;
        }
      }
    }

    if (printing_tree) {
      std::cout << " - type: expanding" << std::endl;
      std::cout << "   id: " << H->node_id << std::endl;
      std::cout << "   pId: "
                << (H->parent == nullptr ? "0"
                                         : std::to_string(H->parent->node_id))
                << std::endl;
      std::cout << "   f_value: " << H->f << std::endl;
    }
    // if(guidance_heuristic.initialized){
    //   // H->reordering_based_on_traffic(N,guidance_heuristic);
    // }
    // create successors at the low-level search

    auto L = H->search_tree.front();
    H->search_tree.pop();
    expand_lowlevel_tree(H, L);

    node_expaned.push_back(H);
    // create successors at the high-level search
    accessed_times += 1;
    const auto res = get_new_config(H, L);
    delete L;  // free
    if (!res) continue;

    // create new configuration
    for (auto a : A) C_new[a->id] = a->v_next;

    // there could be no regret to learn due to swap, so mark the agent
    // accessed;
    if (traffic_op == REGERT_TRAFFIC || traffic_op == TRAINNING_TRAFFIC ||
        traffic_op == LOADING_TRAFFIC || traffic_op == CONTINUE_TRANNING) {
      learning_regret_value(C_next, H->C, C_new);
      learning_traffic_cost(H->C, C_new);
    }
    // check explored list
    const auto iter = EXPLORED.find(C_new);
    if (iter != EXPLORED.end()) {
      // if(iter->second->h != 0){
      //   std::cout<< "!!!!!!!!!!!!!!!!" << std::endl;
      // }
      // case found
      rewrite(H, iter->second, H_goal, OPEN);
      if (OPEN.size() == 1) {
        continue;
      }
      // re-insert or random-restart
      auto H_insert = (MT != nullptr && get_random_float(MT) >= RESTART_RATE)
                          ? iter->second
                          : H_init;
      if (traffic_op == NONE || traffic_op == PRE_TRAFFIC) {
        if (H_goal == nullptr || H_insert->f < H_goal->f) OPEN.push(H_insert);
      } else {
        OPEN.push(H_insert);
      }
    } else {
      // insert new search node
      const auto H_new = new HNode(
          C_new, D, H, H->g + get_edge_cost(H->C, C_new), get_h_value(C_new));
      H_new->set_make_span(H->current_make_span + 1);
      H_new->node_id = num_of_nodes_generated;
      // H_new->reordering_based_on_traffic(N,guidance_heuristic);
      EXPLORED[H_new->C] = H_new;
      if (traffic_op == NONE || traffic_op == PRE_TRAFFIC) {
        if (H_goal == nullptr || H_new->f < H_goal->f) OPEN.push(H_new);
      } else {
        OPEN.push(H_new);
      }
      num_of_nodes_generated++;
      if (printing_tree) {
        std::cout << " - type: generating" << std::endl;
        std::cout << "   id: " << H_new->node_id << std::endl;
        std::cout << "   pId: "
                  << (H_new->parent == nullptr
                          ? "0"
                          : std::to_string(H_new->parent->node_id))
                  << std::endl;
        std::cout << "   f_value: " << H_new->f << std::endl;
      }
    }
  }
  // export_solution_from_HNode(H_last,"deadlock_new_11111.csv");

  // backtrack
  if (H_goal != nullptr) {
    auto H = H_goal;
    while (H != nullptr) {
      solution.push_back(H->C);
      H = H->parent;
    }
    std::reverse(solution.begin(), solution.end());
  }

  // print result
  if (H_goal != nullptr && OPEN.empty()) {
    solver_info(1, "solved optimally, objective: ", objective);
  } else if (H_goal != nullptr) {
    solver_info(1, "solved sub-optimally, objective: ", objective);
  } else if (OPEN.empty()) {
    solver_info(1, "no solution");
  } else {
    solver_info(1, "timeout");
  }

  // logging
  additional_info +=
      "optimal=" + std::to_string(H_goal != nullptr && OPEN.empty()) + "\n";
  additional_info += "objective=" + std::to_string(objective) + "\n";
  additional_info += "loop_cnt=" + std::to_string(loop_cnt) + "\n";
  additional_info += "num_node_gen=" + std::to_string(EXPLORED.size()) + "\n";

  // memory management
  for (auto a : A) delete a;
  for (auto itr : EXPLORED) delete itr.second;

  // traffic_map.export_PIBT_regret_flow_csv("warehouse_traffic.csv");
  return solution;
}

void Planner::learning_regret_value(
    std::vector<std::array<Vertex*, 5>>& C_next_actions, const Config& C_curr,
    Config& C_next)
{
  for (int i = 0; i < C_curr.size(); i++) {
    if (accessed_agents[i] != accessed_times) {
      // not accessed
      continue;
    }
    auto curr_v = C_curr[i];
    int neighbor_size = curr_v->neighbor.size();
    auto next_v = C_next[i];
    bool found = false;
    for (int j = 0; j <= neighbor_size; j++) {
      if (C_next_actions[i][j] == next_v) {
        // found = true;
        break;
      } else {
        traffic_map.increase_regret_on_edge(curr_v->index,
                                            C_next_actions[i][j]->index, 1);
      }
    }
  }
}

void Planner::learning_traffic_cost(const Config& C_from, const Config& C_to)
{
  for (int i = 0; i < N; i++) {
    if (C_from[i]->id == C_to[i]->id && C_to[i]->id == ins->goals[i]->id) {
      // skip wait at target;
      continue;
    }
    traffic_map.increase_regret_single_step(C_from[i]->index, C_to[i]->index);
  }
}

void Planner::rewrite(HNode* H_from, HNode* H_to, HNode* H_goal,
                      std::stack<HNode*>& OPEN)
{
  // update neighbors
  H_from->neighbor.insert(H_to);

  // Dijkstra update
  std::queue<HNode*> Q({H_from});  // queue is sufficient
  while (!Q.empty()) {
    auto n_from = Q.front();
    Q.pop();
    for (auto n_to : n_from->neighbor) {
      auto g_val = n_from->g + get_edge_cost(n_from->C, n_to->C);
      if (g_val < n_to->g) {
        if (printing_tree) {
          std::cout << " - type: rewriting" << std::endl;
          std::cout << "   id: " << n_to->node_id << std::endl;
          std::cout << "   pId: " << std::to_string(n_from->node_id)
                    << std::endl;
          std::cout << "   f_value: " << n_to->f << std::endl;
        }
        if (n_to == H_goal) {
          if (traffic_op != NONE && traffic_op != PRE_TRAFFIC) {
            solver_info(1, "cost update: ", n_to->g, " -> ", g_val);
            n_to->g = g_val;
            n_to->f = n_to->g + n_to->h;
            n_to->parent = n_from;
            n_to->set_make_span(n_from->current_make_span + 1);
            running_traffic_optimization(OPEN, n_to, true);
            return;
          } else {
            solver_info(1, "cost update: ", n_to->g, " -> ", g_val);
          }
        }
        n_to->g = g_val;
        n_to->f = n_to->g + n_to->h;
        n_to->parent = n_from;
        n_to->set_make_span(n_from->current_make_span + 1);
        Q.push(n_to);
        if (H_goal != nullptr && n_to->f < H_goal->f) OPEN.push(n_to);
      }
    }
  }
}

uint Planner::get_edge_cost(const Config& C1, const Config& C2)
{
  if (objective == OBJ_SUM_OF_LOSS) {
    uint cost = 0;
    for (uint i = 0; i < N; ++i) {
      if (C1[i] != ins->goals[i] || C2[i] != ins->goals[i]) {
        cost += 1;
      }
    }
    return cost;
  }

  // default: makespan
  return 1;
}

uint Planner::get_edge_cost(HNode* H_from, HNode* H_to)
{
  return get_edge_cost(H_from->C, H_to->C);
}

uint Planner::get_h_value(const Config& C)
{
  uint cost = 0;
  if (objective == OBJ_MAKESPAN) {
    for (auto i = 0; i < N; ++i) cost = std::max(cost, D.get(i, C[i]));
  } else if (objective == OBJ_SUM_OF_LOSS) {
    for (auto i = 0; i < N; ++i) cost += D.get(i, C[i]);
  }
  return cost;
}

void Planner::expand_lowlevel_tree(HNode* H, LNode* L)
{
  if (L->depth >= N) return;
  const auto i = H->order[L->depth];
  auto C = H->C[i]->neighbor;
  C.push_back(H->C[i]);
  // randomize
  if (MT != nullptr) std::shuffle(C.begin(), C.end(), *MT);
  // insert
  for (auto v : C) H->search_tree.push(new LNode(L, i, v));
}

bool Planner::get_new_config(HNode* H, LNode* L)
{
  // setup cache
  for (auto a : A) {
    // clear previous cache
    if (a->v_now != nullptr && occupied_now[a->v_now->id] == a) {
      occupied_now[a->v_now->id] = nullptr;
    }
    if (a->v_next != nullptr) {
      occupied_next[a->v_next->id] = nullptr;
      a->v_next = nullptr;
    }

    // set occupied now
    a->v_now = H->C[a->id];
    occupied_now[a->v_now->id] = a;
  }

  // add constraints
  for (uint k = 0; k < L->depth; ++k) {
    const auto i = L->who[k];        // agent
    const auto l = L->where[k]->id;  // loc

    // check vertex collision
    if (occupied_next[l] != nullptr) return false;
    // check swap collision
    auto l_pre = H->C[i]->id;
    if (occupied_next[l_pre] != nullptr && occupied_now[l] != nullptr &&
        occupied_next[l_pre]->id == occupied_now[l]->id)
      return false;

    // set occupied_next
    A[i]->v_next = L->where[k];
    occupied_next[l] = A[i];
  }

  // perform PIBT
  for (auto k : H->order) {
    auto a = A[k];
    if (a->v_next == nullptr && !funcPIBT(a)) return false;  // planning failure
  }
  return true;
}

bool Planner::funcPIBT(Agent* ai)
{
  const auto i = ai->id;
  const auto K = ai->v_now->neighbor.size();

  // get candidates for next locations
  for (auto k = 0; k < K; ++k) {
    auto u = ai->v_now->neighbor[k];
    C_next[i][k] = u;
    if (MT != nullptr)
      tie_breakers[u->id] = get_random_float(MT);  // set tie-breaker
    // tie_breakers[u->id] = get_random_float(MT);  // set tie-breaker
  }
  C_next[i][K] = ai->v_now;

  if (traffic_op == INCRE_TRAFFIC || traffic_op == INCRE_TRAFFIC_WITH_TW) {
    if (!guidance_heuristic.initialized) {
      std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
                [&](Vertex* const v, Vertex* const u) {
                  return D.get(i, v) + tie_breakers[v->id] <
                         D.get(i, u) + tie_breakers[u->id];
                });
    } else {
      std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
                [&](Vertex* const v, Vertex* const u) {
                  return guidance_heuristic.get_Astar_heuristic(i, v->id) +
                             tie_breakers[v->id] <
                         guidance_heuristic.get_Astar_heuristic(i, u->id) +
                             tie_breakers[u->id];
                });
    }
  } else if (traffic_op == ONLINE_TRAFFIC || traffic_op == PRE_TRAFFIC ||
             traffic_op == ONLINE_TRAFFIC_TW ||
             traffic_op == INCRE_PLUS_ONLINE_TRAFFIC) {
    if (traffic_op == PRE_TRAFFIC) {
      if (!guidance_heuristic.initialized) {
        std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
                  [&](Vertex* const v, Vertex* const u) {
                    return D.get(i, v) + tie_breakers[v->id] <
                           D.get(i, u) + tie_breakers[u->id];
                  });
      } else {
        if (revised_path[i].empty()) {
          std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
                    [&](Vertex* const v, Vertex* const u) {
                      return D.get(i, v) + tie_breakers[v->id] <
                             D.get(i, u) + tie_breakers[u->id];
                    });
        } else {
          std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
                    [&](Vertex* const v, Vertex* const u) {
                      return guidance_heuristic.get_heuristic(i, v->index) +
                                 tie_breakers[v->id] <
                             guidance_heuristic.get_heuristic(i, u->index) +
                                 tie_breakers[u->id];
                    });
        }
        // std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
        //       [&](Vertex* const v, Vertex* const u) {
        //         return guidance_heuristic.get_heuristic(i,v->index) <
        //         guidance_heuristic.get_heuristic(i,u->index) ;
        //       });
      }
    }
  } else if (traffic_op == REGERT_TRAFFIC || traffic_op == TRAINNING_TRAFFIC ||
             traffic_op == LOADING_TRAFFIC || traffic_op == CONTINUE_TRANNING) {
    if (!guidance_heuristic.initialized) {
      std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
                [&](Vertex* const v, Vertex* const u) {
                  return D.get(i, v) + tie_breakers[v->id] <
                         D.get(i, u) + tie_breakers[u->id];
                });
    } else {
      std::sort(
          C_next[i].begin(), C_next[i].begin() + K + 1,
          [&](Vertex* const v, Vertex* const u) {
            // double a1 = guidance_heuristic.get_regret_heuristic(i, v->id) +
            // traffic_map.get_regret_cost(ai->v_now->index,v->index) +
            // tie_breakers[v->id]; double a2 =
            // guidance_heuristic.get_regret_heuristic(i, u->id) +
            // traffic_map.get_regret_cost(ai->v_now->index,u->index) +
            // tie_breakers[u->id];
            double a1 = guidance_heuristic.get_regret_heuristic(i, v->id) +
                        tie_breakers[v->id];
            double a2 = guidance_heuristic.get_regret_heuristic(i, u->id) +
                        tie_breakers[u->id];
            return a1 < a2;
          });
    }
  } else {
    // sort
    std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
              [&](Vertex* const v, Vertex* const u) {
                return D.get(i, v) + tie_breakers[v->id] <
                       D.get(i, u) + tie_breakers[u->id];
              });
  }
  // mark the agent as accessed ;
  accessed_agents[i] = accessed_times;
  Agent* swap_agent = swap_possible_and_required(ai);
  if (swap_agent != nullptr) {
    if (swap_based_on_original_distance) {
      std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
                [&](Vertex* const v, Vertex* const u) {
                  return D.get(i, v) + tie_breakers[v->id] <
                         D.get(i, u) + tie_breakers[u->id];
                });
    }

    std::reverse(C_next[i].begin(), C_next[i].begin() + K + 1);
  }

  // main operation
  for (auto k = 0; k < K + 1; ++k) {
    auto u = C_next[i][k];

    // avoid vertex conflicts
    if (occupied_next[u->id] != nullptr) continue;

    auto& ak = occupied_now[u->id];

    // avoid swap conflicts
    if (ak != nullptr && ak->v_next == ai->v_now) continue;

    // reserve next location
    occupied_next[u->id] = ai;
    ai->v_next = u;

    // priority inheritance
    if (ak != nullptr && ak != ai && ak->v_next == nullptr && !funcPIBT(ak))
      continue;

    // success to plan next one step
    // pull swap_agent when applicable
    if (k == 0 && swap_agent != nullptr && swap_agent->v_next == nullptr &&
        occupied_next[ai->v_now->id] == nullptr) {
      swap_agent->v_next = ai->v_now;
      occupied_next[swap_agent->v_next->id] = swap_agent;
    }
    return true;
  }

  // failed to secure node
  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;
  return false;
}

Agent* Planner::swap_possible_and_required(Agent* ai)
{
  const auto i = ai->id;
  // ai wanna stay at v_now -> no need to swap
  if (C_next[i][0] == ai->v_now) return nullptr;

  // usual swap situation, c.f., case-a, b
  auto aj = occupied_now[C_next[i][0]->id];
  if (aj != nullptr && aj->v_next == nullptr &&
      is_swap_required(ai->id, aj->id, ai->v_now, aj->v_now) &&
      is_swap_possible(aj->v_now, ai->v_now)) {
    return aj;
  }

  // for clear operation, c.f., case-c
  for (auto u : ai->v_now->neighbor) {
    auto ak = occupied_now[u->id];
    if (ak == nullptr || C_next[i][0] == ak->v_now) continue;
    if (is_swap_required(ak->id, ai->id, ai->v_now, C_next[i][0]) &&
        is_swap_possible(C_next[i][0], ai->v_now)) {
      return ak;
    }
  }

  return nullptr;
}

// simulate whether the swap is required
bool Planner::is_swap_required(const uint pusher, const uint puller,
                               Vertex* v_pusher_origin, Vertex* v_puller_origin)
{
  if (!swap_based_on_original_distance) {
    if (traffic_op == INCRE_TRAFFIC || traffic_op == INCRE_TRAFFIC_WITH_TW) {
      if (guidance_heuristic.initialized) {
        return is_swap_required_gudiance_Astar_version(
            pusher, puller, v_pusher_origin, v_puller_origin);
      }
    } else if (traffic_op == ONLINE_TRAFFIC || traffic_op == PRE_TRAFFIC ||
               traffic_op == ONLINE_TRAFFIC_TW ||
               traffic_op == INCRE_PLUS_ONLINE_TRAFFIC) {
      if (guidance_heuristic.initialized) {
        return is_swap_required_gudiance_version(
            pusher, puller, v_pusher_origin, v_puller_origin);
      }
    } else if (traffic_op == REGERT_TRAFFIC ||
               traffic_op == TRAINNING_TRAFFIC ||
               traffic_op == LOADING_TRAFFIC ||
               traffic_op == CONTINUE_TRANNING) {
      if (guidance_heuristic.initialized) {
        return is_swap_required_gudiance_regret_version(
            pusher, puller, v_pusher_origin, v_puller_origin);
      }
    }
  }
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex* tmp = nullptr;
  while (D.get(pusher, v_puller) < D.get(pusher, v_pusher)) {
    auto n = v_puller->neighbor.size();
    // remove agents who need not to move
    for (auto u : v_puller->neighbor) {
      auto a = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && a != nullptr && ins->goals[a->id] == u)) {
        --n;
      } else {
        tmp = u;
      }
    }
    if (n >= 2) return false;  // able to swap
    if (n <= 0) break;
    v_pusher = v_puller;
    v_puller = tmp;
  }

  // judge based on distance
  return (D.get(puller, v_pusher) < D.get(puller, v_puller)) &&
         (D.get(pusher, v_pusher) == 0 ||
          D.get(pusher, v_puller) < D.get(pusher, v_pusher));
}

// simulate whether the swap is required
bool Planner::is_swap_required_gudiance_version(const uint pusher,
                                                const uint puller,
                                                Vertex* v_pusher_origin,
                                                Vertex* v_puller_origin)
{
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex* tmp = nullptr;
  while (guidance_heuristic.get_heuristic(pusher, v_puller->index) <
         guidance_heuristic.get_heuristic(pusher, v_pusher->index)) {
    auto n = v_puller->neighbor.size();
    // remove agents who need not to move
    for (auto u : v_puller->neighbor) {
      auto a = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && a != nullptr && ins->goals[a->id] == u)) {
        --n;
      } else {
        tmp = u;
      }
    }
    if (n >= 2) return false;  // able to swap
    if (n <= 0) break;
    v_pusher = v_puller;
    v_puller = tmp;
  }

  // judge based on distance
  return (guidance_heuristic.get_heuristic(puller, v_pusher->index) <
          guidance_heuristic.get_heuristic(puller, v_puller->index)) &&
         (guidance_heuristic.get_heuristic(pusher, v_pusher->index) == 0 ||
          guidance_heuristic.get_heuristic(pusher, v_puller->index) <
              guidance_heuristic.get_heuristic(pusher, v_pusher->index));
}

bool Planner::is_swap_required_gudiance_regret_version(const uint pusher,
                                                       const uint puller,
                                                       Vertex* v_pusher_origin,
                                                       Vertex* v_puller_origin)
{
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex* tmp = nullptr;
  while (guidance_heuristic.get_regret_heuristic(pusher, v_puller->id) <
         guidance_heuristic.get_regret_heuristic(pusher, v_pusher->id)) {
    auto n = v_puller->neighbor.size();
    // remove agents who need not to move
    for (auto u : v_puller->neighbor) {
      auto a = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && a != nullptr && ins->goals[a->id] == u)) {
        --n;
      } else {
        tmp = u;
      }
    }
    if (n >= 2) return false;  // able to swap
    if (n <= 0) break;
    v_pusher = v_puller;
    v_puller = tmp;
  }

  // judge based on distance
  return (guidance_heuristic.get_regret_heuristic(puller, v_pusher->id) <
          guidance_heuristic.get_regret_heuristic(puller, v_puller->id)) &&
         (guidance_heuristic.get_regret_heuristic(pusher, v_pusher->id) == 0 ||
          guidance_heuristic.get_regret_heuristic(pusher, v_puller->id) <
              guidance_heuristic.get_regret_heuristic(pusher, v_pusher->id));
}

// simulate whether the swap is required
bool Planner::is_swap_required_gudiance_Astar_version(const uint pusher,
                                                      const uint puller,
                                                      Vertex* v_pusher_origin,
                                                      Vertex* v_puller_origin)
{
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex* tmp = nullptr;
  while (guidance_heuristic.get_Astar_heuristic(pusher, v_puller->id) <
         guidance_heuristic.get_Astar_heuristic(pusher, v_pusher->id)) {
    auto n = v_puller->neighbor.size();
    // remove agents who need not to move
    for (auto u : v_puller->neighbor) {
      auto a = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && a != nullptr && ins->goals[a->id] == u)) {
        --n;
      } else {
        tmp = u;
      }
    }
    if (n >= 2) return false;  // able to swap
    if (n <= 0) break;
    v_pusher = v_puller;
    v_puller = tmp;
  }

  // judge based on distance
  return (guidance_heuristic.get_Astar_heuristic(puller, v_pusher->id) <
          guidance_heuristic.get_Astar_heuristic(puller, v_puller->id)) &&
         (guidance_heuristic.get_Astar_heuristic(pusher, v_pusher->id) == 0 ||
          guidance_heuristic.get_Astar_heuristic(pusher, v_puller->id) <
              guidance_heuristic.get_Astar_heuristic(pusher, v_pusher->id));
}

// simulate whether the swap is possible
bool Planner::is_swap_possible(Vertex* v_pusher_origin, Vertex* v_puller_origin)
{
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex* tmp = nullptr;
  while (v_puller != v_pusher_origin) {  // avoid loop
    auto n = v_puller->neighbor.size();  // count #(possible locations) to pull
    for (auto u : v_puller->neighbor) {
      auto a = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && a != nullptr && ins->goals[a->id] == u)) {
        --n;  // pull-impossible with u
      } else {
        tmp = u;  // pull-possible with u
      }
    }
    if (n >= 2) return true;  // able to swap
    if (n <= 0) return false;
    v_pusher = v_puller;
    v_puller = tmp;
  }
  return false;
}

std::ostream& operator<<(std::ostream& os, const Objective obj)
{
  if (obj == OBJ_NONE) {
    os << "none";
  } else if (obj == OBJ_MAKESPAN) {
    os << "makespan";
  } else if (obj == OBJ_SUM_OF_LOSS) {
    os << "sum_of_loss";
  }
  return os;
}

std::ostream& operator<<(std::ostream& os, const Traffic_OP traffic)
{
  if (traffic == NONE) {
    os << "none";
  } else if (traffic == PRE_TRAFFIC) {
    os << "pre-traffic";
  } else if (traffic == ONLINE_TRAFFIC) {
    os << "online-traffic";
  } else if (traffic == ONLINE_TRAFFIC_TW) {
    os << "online-traffic-with-tw";
  } else if (traffic == INCRE_TRAFFIC) {
    os << "incre-traffic";
  } else if (traffic == INCRE_TRAFFIC_WITH_TW) {
    os << "incre-traffic-with-tw";
  }
  return os;
}