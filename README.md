# Final Project

This project is a C++ flocking simulation built around a position-based dynamics style update loop. Each bird is treated as a particle with predicted positions, projected collision constraints, and flocking forces for separation, alignment, cohesion, obstacle avoidance, and predator evasion.

## Features

- boid-style local steering rules
- multiple predator chase scenarios
- predator catch behavior that removes birds when they are reached
- goal-directed flock movement
- particle non-overlap constraint
- obstacle collision projection
- several obstacle layouts and test scenarios
- bounded world with soft steering back toward the simulation region
- smooth orientation aligned to velocity
- optional debug overlays for velocity, neighbor ranges, obstacle sensing, and predator sensing
- interactive keyboard controls for tuning parameters live

## Build

The project expects local GLFW and Eigen paths similar to the course machine setup.

```bash
export GLFW_DIR=/Users/sahilkasturi/Desktop/csce450/libs/glfw
export EIGEN3_INCLUDE_DIR=/opt/homebrew/opt/eigen/include/eigen3
cmake -S . -B build
cmake --build build -j4
```

## Run

```bash
./build/FinalProject
```

## Controls

- `Space`: pause/resume
- `R`: reset
- `N` / `B`: next or previous scenario
- `P`: toggle predator
- `V`: toggle goal-directed movement
- `D`: toggle debug overlays
- `Up` / `Down`: increase or decrease flock size
- `Q` / `A`: separation weight up/down
- `W` / `S`: alignment weight up/down
- `E` / `C`: cohesion weight up/down
- `T` / `G`: predator avoidance weight up/down
- `Y` / `H`: max speed up/down

## Simulation Structure

Each fixed timestep does the following:

1. Update predator steering for the current scenario.
2. Compute per-bird flocking, goal, and avoidance accelerations.
3. Predict particle positions.
4. Project constraints for bird-bird spacing, obstacle collisions, and world boundaries.
5. Reconstruct velocities from corrected positions.

## Scenarios

- `Open Field`: balanced flocking, a single predator, and a light obstacle layout.
- `Obstacle Course`: rows of obstacles that force the flock to navigate through gaps toward a goal.
- `Split And Regroup`: central obstacles and two predators that encourage the flock to scatter and reform.
- `Dense Obstacles`: a harder navigation test with many obstacles and two predators.

These scenarios are designed to demonstrate emergent group motion, obstacle response, predator evasion, and goal-directed navigation without pre-made animation clips.
