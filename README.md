# LaCAM* + Lightweight Traffic Map (LTM)
[![MIT License](http://img.shields.io/badge/license-MIT-blue.svg?style=flat)](LICENCE.txt)

This is the code repository for the paper:

**["A Lightweight Traffic Map for Efficient Anytime LaCAM*"](https://arxiv.org/abs/2603.07891)**
Bojie Shen, Yue Zhang, Zhe Chen, Daniel Harabor

> Multi-Agent Path Finding (MAPF) seeks collision-free paths for teams of agents and has a wide range of practical applications. LaCAM\*, an anytime configuration-based solver, currently represents the state-of-the-art. Recent work has explored using guidance paths to steer LaCAM\* toward configurations that avoid traffic congestion, thereby improving solution quality. However, existing approaches rely on Frank–Wolfe–style optimisation that repeatedly invokes single-agent search before executing LaCAM\*, creating substantial computational overhead for large-scale problems. Moreover, the guide path is static and only helpful for finding the first solution. To overcome these limitations, we propose a new approach that exploits LaCAM\*'s ability to construct a **dynamic, lightweight traffic map** during search. Experiments show that our method achieves higher solution quality than state-of-the-art guidance-path approaches in two MAPF variants.

## Key Features

- **Lightweight Traffic Map (LTM):** a dynamic directed weighted graph built **online during search** from PIBT's execution history (committed & blocked actions) — no offline precomputation, no training data. PIBT's distance heuristic is replaced by distances on the live LTM.
- **Shared anytime loop:** run bounded LaCAM\* guided by the current LTM → keep the best solution → update the LTM from PIBT history → select a restart node → repeat.
- **Two MAPF settings, same loop:**
  - *One-shot MAPF* — a single time budget; the first iteration is plain LaCAM\*, then the solver restarts (mostly from the root) to drive final solution quality as high as possible.
  - *Planning-and-execution MAPF* — repeated short planning windows; each window commits the next X actions, reuses the search tree, restarts from the committed configuration, and refreshes the LTM.
- **Superior anytime performance:** lower sum-of-loss than LaCAM\*+TO and LaCAM\*+SUO (one-shot) and than PIE (planning-and-execution), with the gap widening as agent density grows.

## Building

**Requirements:** [CMake](https://cmake.org/) ≥ 3.16, C++17 compiler.

Clone the repository with submodules:

```sh
git clone --recursive https://github.com/bshen95/lacam-ltm.git && cd lacam-ltm
```

Build:

```sh
cmake -B build && make -C build
```

Alternatively, a Docker environment is available in `assets/`.

## Usage

### One-shot MAPF (LaCAM\* + LTM)

A single run with a fixed time budget; the LTM is built from scratch online and the solver keeps improving the incumbent until the deadline (`-f 7`):

```sh
# 400 agents on random-32-32-20, 30s budget, sum-of-loss objective
build/main -m scripts/map/random-32-32-20.map \
           -i scripts/scen/scen-random/random-32-32-20-random-1.scen \
           -N 400 -t 30 -O 2 -f 7 -v 1 -o build/result.txt
```

### Planning-and-execution MAPF (LaCAM\* + LTM)

Repeated short planning windows (`-f 12`): each window plans for `-p` milliseconds, commits the next `-c` actions for execution, then replans from the committed configuration with a refreshed LTM. The paper evaluates window budgets E ∈ {0.1 s, 0.5 s} and commit steps X ∈ {5, 10, 20}:

```sh
# E = 0.1s planning window, commit X = 5 actions per window
build/main -m scripts/map/random-64-64-20.map \
           -i scripts/scen/scen-random/random-64-64-20-random-1.scen \
           -N 800 -t 60 -O 2 -f 12 -p 100 -c 5 -v 1 -o build/result.txt
```

In this setting the run ends as soon as all agents reach their goals; `-t` only needs to be a generous upper bound (a too-small `-t` cuts execution short and yields an incomplete solution).

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
| `-f, --traffic` | Solver mode: `0` = LaCAM\* baseline, `7` = LTM one-shot, `12` = LTM planning-and-execution | `0` |
| `-p, --planning_time` | Planning-and-execution only: planning window per replan, in ms | `1000` |
| `-c, --commit_steps` | Planning-and-execution only: actions committed per window (X) | `1` |
| `-r, --restart_rate` | Probability of restarting from the root when revisiting a configuration | `0.001` |
| `-v, --verbose` | Verbosity level | `0` |
| `-o, --output` | Output file path | `./build/result.txt` |
| `-l, --log_short` | Short log format | `false` |
| `-g, --generate_instance` | Export instance and exit | `false` |

Other `--traffic` values select internal/experimental guidance variants and are not part of the paper.

## Reproducing the Paper Experiments

Both settings are evaluated on 8 grid maps from the [MAPF benchmark](https://movingai.com/benchmarks/mapf.html) with 25 random scenarios per map, sum-of-loss objective, and up to 2000 agents:

`random-32-32-20`, `random-64-64-20`, `empty-32-32`, `empty-48-48`, `maze-32-32-4`, `room-64-64-8`, `warehouse-10-20-10-2-1`, `warehouse-10-20-10-2-2`

**One-shot** (Figure 1): 30-second budget per instance, e.g.

```sh
for s in $(seq 1 25); do
  build/main -m scripts/map/empty-48-48.map \
             -i scripts/scen/scen-random/empty-48-48-random-$s.scen \
             -N 1000 -t 30 -O 2 -f 7 -o build/result.txt
done
```

**Planning-and-execution** (Figure 4): vary the window budget (`-p 100` for E = 0.1 s, `-p 500` for E = 0.5 s) and commit steps (`-c 5/10/20`), e.g.

```sh
for s in $(seq 1 25); do
  build/main -m scripts/map/room-64-64-8.map \
             -i scripts/scen/scen-random/room-64-64-8-random-$s.scen \
             -N 1200 -t 60 -O 2 -f 12 -p 100 -c 10 -o build/result.txt
done
```

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
└── third_party/              # argparse, googletest (submodules)
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
