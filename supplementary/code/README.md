# Robust-Adaptive Control of Linear Systems: beyond Quadratic Costs

This repository contains the source code used for the paper *Robust-Adaptive Control of Linear Systems: beyond Quadratic Costs*.
It is written in Python 3.

## Implementations of the main algorithmic steps

* The confidence ellipsoid (8) is implemented in the `ellipsoid()` method in `code/rl_agents/agents/tree_search/robust_epc.py`
* The corresponding polytope (9) is implemented in the `polytope()` method of `code/rl_agents/agents/tree_search/robust_epc.py`
* The simple predictor of (10) is implemented in the `step_simple_predictor()` method of `code/rl_agents/agents/common/interval.py`
* The enhanced predictor of (11) is implemented in the `step_interval_predictor()` method of `code/rl_agents/agents/common/interval.py`
* The pessimistic reward of (12) is implemented in the `pessimistic_reward()` method of `code/obstacle_env/envs/obstacle.py` and the `check_collision()` method of `code/highway_env/vehicle/uncertainty/prediction.py`
* The robust upper-bound of (15) is implemented in the `RobustNode` class of `code/rl_agents/agents/tree_search/robust.py`

## Installation

Install the required packages with:

```pip3 install requirements.txt```

## Instructions

The experiments can be reproduced by running:

```
cd scripts
```

### Obstacle avoidance

```
python experiments.py configs/ObstacleEnv/env.json configs/ObstacleEnv/agents/oracle.json
python experiments.py configs/ObstacleEnv/env.json configs/ObstacleEnv/agents/nominal.json
python experiments.py configs/ObstacleEnv/env.json configs/ObstacleEnv/agents/robust-epc.json
```

### Motion planning for an autonomous vehicle

```
python experiments.py configs/IntersectionEnv/env.json configs/IntersectionEnv/agents/oracle.json
python experiments.py configs/IntersectionEnv/env.json configs/IntersectionEnv/agents/known_route.json
python experiments.py configs/IntersectionEnv/env.json configs/IntersectionEnv/agents/minimum_error_route.json
python experiments.py configs/IntersectionEnv/env.json configs/IntersectionEnv/agents/robust-epc.json
```

The results will appear in the `scripts/out` directory.