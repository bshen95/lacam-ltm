#include "../include/lacam2.hpp"

#include <filesystem>

static std::string filename_stem(const std::string& path)
{
#ifdef __cpp_lib_filesystem
  std::filesystem::path p(path);
  return p.stem().string();
#else
  auto pos = path.find_last_of("/\\");
  std::string name = (pos == std::string::npos) ? path : path.substr(pos + 1);
  auto dot = name.find_last_of('.');
  return (dot == std::string::npos) ? name : name.substr(0, dot);
#endif
}

Solution solve(const Instance& ins, std::string& additional_info,
               const int verbose, const Deadline* deadline, std::mt19937* MT,
               const Objective objective, const Traffic_OP traffic,
               const std::string map_name, const float restart_rate)
{
  auto planner =
      Planner(&ins, deadline, MT, verbose, objective, traffic, restart_rate);
  if (traffic == LOADING_TRAFFIC || traffic == CONTINUE_TRANNING) {
    std::string base = filename_stem(map_name);
    if (std::filesystem::exists(base + ".csv")) {
      planner.traffic_map.import_PIBT_regret_flow_csv(base + ".csv");
      planner.load_traffic_csv = true;
    } else {
      planner.load_traffic_csv = false;
    }
  }
  if (traffic == CONTINUE_TRANNING) {
    planner.traffic_map.snapshot_flow_map();
  }
  auto solution = planner.solve(additional_info);
  if (traffic == TRAINNING_TRAFFIC || traffic == CONTINUE_TRANNING) {
    std::string base = filename_stem(map_name);
    planner.traffic_map.export_PIBT_regret_flow_csv(base + ".csv");
  }
  return solution;
}
