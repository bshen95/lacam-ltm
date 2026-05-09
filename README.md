# LaCAM* + Lightweight Traffic Map (LTM)
[![MIT License](http://img.shields.io/badge/license-MIT-blue.svg?style=flat)](LICENCE.txt)

This is the code repository for the paper:

**["A Lightweight Traffic Map for Efficient Anytime LaCAM*"](https://arxiv.org/abs/2603.07891)**
Bojie Shen, Yue Zhang, Zhe Chen, Daniel Harabor

> Multi-Agent Path Finding (MAPF) seeks collision-free paths for teams of agents and has a wide range of practical applications. LaCAM\*, an anytime configuration-based solver, currently represents the state-of-the-art. Recent work has explored using guidance paths to steer LaCAM\* toward configurations that avoid traffic congestion, thereby improving solution quality. However, existing approaches rely on Frank–Wolfe–style optimisation that repeatedly invokes single-agent search before executing LaCAM\*, creating substantial computational overhead for large-scale problems. Moreover, the guide path is static and only helpful for finding the first solution. To overcome these limitations, we propose a new approach that exploits LaCAM\*'s ability to construct a **dynamic, lightweight traffic map** during search. Experiments show that our method achieves higher solution quality than state-of-the-art guidance-path approaches in two MAPF variants.

## Key Features

- **Lightweight Traffic Map (LTM):** A dynamic directed weighted graph that captures congestion from PIBT execution history (committed & blocked actions), updated online during search — no expensive offline precomputation required.
- **Frequent Restarts:** Modified LaCAM\* with early termination conditions and configurable node budgets, enabling rapid iterative improvement.
- **Two MAPF Settings:**
  - *One-shot MAPF* — given a fixed time budget, return the best solution found.
  - *Planning & Execution MAPF* — repeatedly plan under short time windows while committing actions for execution.
- **Superior Anytime Performance:** Faster convergence and higher solution quality than LaCAM\*+TO (Traffic Optimisation), LaCAM\*+SUO (Space Utilization Optimisation), and PIE.

## Building

**Requirements:** [CMake](https://cmake.org/) ≥ 3.16, C++17 compiler.

Clone the repository with submodules:

```sh
git clone --recursive https://github.com/<your-username>/lacam2.git && cd lacam2
```

Build:

```sh
cmake -B build && make -C build
```

Alternatively, a Docker environment is available in `assets/`.

## Usage

### One-shot MAPF with LTM

Run with sum-of-loss optimisation and the LTM traffic mode:

```sh
# 600 agents on random-32-32-20, 30s time limit, LTM enabled (--traffic 10)
build/main -m scripts/map/random-32-32-20.map -t 30 -N 600 --objective 2 -v 2 --traffic 10
```

### Baseline LaCAM* (no traffic guidance)

```sh
build/main -m assets/random-32-32-20.map -N 400 -v 1
```

### All CLI Parameters

| Flag | Description | Default |
|------|-------------|---------|
| `-m, --map` | Map file (required) | — |
| `-i, --scen` | Scenario file | `""` (random) |
| `-N, --num` | Number of agents (required) | — |
| `-s, --seed` | Random seed | `0` |
| `-t, --time_limit_sec` | Time limit in seconds | `3` |
| `-O, --objective` | Objective: `0` = none, `1` = makespan, `2` = sum-of-loss | `0` |
| `-f, --traffic` | Traffic mode (see below) | `0` |
| `-v, --verbose` | Verbosity level | `0` |
| `-p, --planning_time` | Planning time budget in ms | `1000` |
| `-c, --commit_steps` | Commitment horizon (actions to commit per window) | `1` |
| `-r, --restart_rate` | Restart rate | `0.001` |
| `-o, --output` | Output file path | `./build/result.txt` |
| `-l, --log_short` | Short log format | `false` |
| `-g, --generate_instance` | Export instance and exit | `false` |

#### Traffic Modes (`--traffic`)

| Value | Mode |
|-------|------|
| `0` | None (original LaCAM\*) |
| `1` | Pre-traffic |
| `2` | Online traffic |
| `3` | Online traffic with time windows |
| `4` | Incremental traffic |
| `5` | Incremental traffic with time windows |
| `6` | Incremental + online traffic |
| `7` | Regret traffic |
| `8` | Training traffic |
| `9` | Loading traffic |
| `10` | **LTM (our method)** |

## Reproducing Experiments

### One-shot MAPF Experiments

We evaluate on 8 grid maps from the [MAPF benchmark](https://movingai.com/benchmarks/mapf.html) with 25 random instances per map and a 30-second time limit:

```sh
# Run the full experiment suite
bash larger_training.sh
```

### Maps Used

| Map | Agents |
|-----|--------|
| `random-32-32-20` | up to 600 |
| `random-64-64-20` | up to 1000 |
| `empty-32-32` | up to 600 |
| `empty-48-48` | up to 1000 |
| `maze-32-32-4` | up to 400 |
| `room-64-64-8` | up to 1000 |
| `warehouse-10-20-10-2-2` | up to 1000 |
| `warehouse-10-20-10-2-1` | up to 1000 |

## Visualiser

This repository is compatible with [@Kei18/mapf-visualizer](https://github.com/kei18/mapf-visualizer):

```sh
mapf-visualizer scripts/map/random-32-32-20.map build/result.txt
```

## Project Structure

```
├── main.cpp                  # Entry point & CLI argument parsing
├── lacam2/
│   ├── include/
│   │   ├── planner.hpp       # Core LaCAM* planner
│   │   ├── traffic_map.hpp   # Lightweight Traffic Map implementation
│   │   ├── guidance_heuristic.hpp
│   │   ├── heap.hpp          # Priority queue
│   │   ├── modified_Astar.hpp
│   │   ├── graph.hpp         # Graph representation
│   │   ├── instance.hpp      # Problem instance
│   │   ├── dist_table.hpp    # BFS distance table
│   │   ├── post_processing.hpp
│   │   └── utils.hpp
│   └── src/                  # Corresponding implementations
├── scripts/                  # Evaluation scripts (Julia), maps, scenarios
├── assets/                   # Sample maps, scenarios, Docker setup
├── tests/                    # Google Test unit tests
├── third_party/              # argparse, googletest (submodules)
├── training.sh               # Single-seed experiment script
└── larger_training.sh        # Multi-seed experiment script (25 seeds)
```

## Notes

- Maps and scenarios in `assets/` and `scripts/map/` are from [MAPF benchmarks](https://movingai.com/benchmarks/mapf.html).
- This codebase is built on top of the original [LaCAM*](https://github.com/Kei18/lacam2) by Keisuke Okumura.
- Auto formatting with clang-format:
  ```sh
  git config core.hooksPath .githooks && chmod a+x .githooks/pre-commit
  ```

## Citation

If you use this code in your research, please cite:

```bibtex
@article{shen2026lightweight,
  title={A Lightweight Traffic Map for Efficient Anytime LaCAM*},
  author={Shen, Bojie and Zhang, Yue and Chen, Zhe and Harabor, Daniel},
  journal={arXiv preprint arXiv:2603.07891},
  year={2026}
}
```

## Licence

This software is released under the MIT License, see [LICENCE.txt](LICENCE.txt).

