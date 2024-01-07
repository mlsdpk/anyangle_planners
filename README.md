# Algorithms for Any-angle Pathfinding

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
cmake  -DBUILD_APPS=ON ..
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
            "id": "MovingAILabScenario"
        },
        "num_of_runs": 100
    },
    "GridMap": {
        "connectivity": "four",
        "distance_metric": "Euclidean"
    },
    "MovingAILabScenario": {
        "scenario": "Berlin_0_256"
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