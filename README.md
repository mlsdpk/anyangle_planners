# Algorithms for Any-angle Pathfinding
![MIT](https://img.shields.io/badge/license-MIT-blue.svg)
![CI](https://github.com/mlsdpk/anyangle_planners/workflows/build/badge.svg)

This repository contains implementations of popular 2D pathfinding algorithms from the literature, along with convenient scripts for benchmarking and visualization. The example 2D benchmark grid maps and problems are from [Nathan Sturtevant's Moving AI Lab](https://www.movingai.com/).

# Why this?

This repository was created while I was completing my Bachelor's dissertation. Unfortunately, the project itself is still in progress, and hopefully, I can release the first usable version in the near future for the community to accelerate path-finding research.

# Installation

## Building from source

```bash
git clone --recursive git@github.com:mlsdpk/anyangle_planners.git
cd anyangle_planners
mkdir build && cd build
cmake ..
make
```

if you want to build example applications (i.e., benchmarking and visualization), you must add `-DBUILD_APPS=ON` to the `cmake` call above:

```bash
cmake  -DBUILD_APPS=ON -DCMAKE_BUILD_TYPE=Release ..
```

# Example Applications

## Benchmarking

```json
{
    "experiment": {
        "planners": [
            "Dijkstra",
            "AStar",
            "ThetaStar"
        ],
        "environment": {
            "type": "GridMap",
            "id": "MovingAILabScenario",
            "MovingAILabScenario": {
                "scenario_file_name": "../maps/street-scen/1024/Boston_0_1024.map.scen"
            }
        },
        "num_of_runs": 100
    },
    "GridMap": {
        "connectivity": "four",
        "distance_metric": "Euclidean"
    }
}
```

```
$ ./apps/benchmark ../experiments/benchmark.json

+-----------------------------------------------------------------------+
|       Algorithms for Any-angle Pathfinding written in Modern C++      |
|              https://github.com/mlsdpk/anyangle_planners              |
+-----------------------------------------------------------------------+

 Using benchmark template json file from ../experiments/benchmark.json

+-------------+---------------------+
|        Type | GridMap             |
+-------------+---------------------+
|          ID | MovingAILabScenario |
+-------------+---------------------+
| No: of runs | 100                 |
+-------------+---------------------+
|    Planners | Dijkstra            |
|             | AStar               |
|             | ThetaStar           |
+-------------+---------------------+
```
