#pragma once

#include <Eigen/Dense>

#include <vector>

enum class Scenario
{
    OpenField = 0,
    ObstacleCourse,
    SplitAndRegroup,
    DenseObstacles,
};

struct Bird
{
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    Eigen::Vector3f predictedPosition = Eigen::Vector3f::Zero();
    Eigen::Vector3f previousPosition = Eigen::Vector3f::Zero();
    Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
    Eigen::Vector3f heading = Eigen::Vector3f::UnitX();
};

struct Obstacle
{
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    float radius = 1.0f;
};

struct Predator
{
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
    Eigen::Vector3f heading = Eigen::Vector3f::UnitX();
    bool enabled = true;
};

struct SimulationParameters
{
    int flockSize = 120;
    float worldHalfExtent = 22.0f;
    float birdRadius = 0.22f;
    float maxSpeed = 7.5f;
    float maxAcceleration = 22.0f;
    float neighborRadius = 4.0f;
    float separationRadius = 1.1f;
    float obstacleSenseRadius = 4.5f;
    float predatorSenseRadius = 7.0f;
    float separationWeight = 17.0f;
    float alignmentWeight = 5.0f;
    float cohesionWeight = 3.5f;
    float obstacleWeight = 22.0f;
    float predatorAvoidWeight = 34.0f;
    float goalWeight = 4.5f;
    float boundaryWeight = 9.0f;
    float damping = 0.992f;
    float predatorMaxSpeed = 8.0f;
    float predatorAcceleration = 19.0f;
    float predatorCatchupWeight = 10.0f;
    float predatorCatchRadius = 0.85f;
    int solverIterations = 3;
};

class Simulation
{
public:
    Simulation();

    void reset();
    void resizeFlock(int newSize);
    void update(float dt);

    void setPaused(bool paused);
    bool isPaused() const;

    void togglePredator();
    void setPredatorEnabled(bool enabled);
    void toggleGoal();
    void nextScenario();
    void previousScenario();
    void setScenario(Scenario scenario);

    SimulationParameters &params();
    const SimulationParameters &params() const;
    const std::vector<Bird> &birds() const;
    const std::vector<Obstacle> &obstacles() const;
    const Predator &predator() const;
    const std::vector<Predator> &predators() const;
    const Eigen::Vector3f &goal() const;
    bool goalEnabled() const;
    Scenario scenario() const;
    const char *scenarioName() const;

private:
    void configureScenario();
    void resetPredators();
    Eigen::Vector3f randomPointInWorld(float margin) const;
    Eigen::Vector3f randomUnitVector() const;
    Eigen::Vector3f clampMagnitude(const Eigen::Vector3f &value, float maxMagnitude) const;
    Eigen::Vector3f computeBirdAcceleration(std::size_t index) const;
    Eigen::Vector3f computeBoundaryForce(const Eigen::Vector3f &position) const;
    void integrateBirds(float dt);
    void projectConstraints();
    void solveBirdSeparation();
    void solveObstacleCollisions();
    void solveBoundaryCollisions();
    void finalizeBirds(float dt);
    void updatePredators(float dt);
    void removeCaughtBirds();

    SimulationParameters m_params;
    std::vector<Bird> m_birds;
    std::vector<Obstacle> m_obstacles;
    std::vector<Predator> m_predators;
    Eigen::Vector3f m_goal = Eigen::Vector3f(15.0f, 0.0f, 15.0f);
    Scenario m_scenario = Scenario::OpenField;
    bool m_paused = false;
    bool m_predatorsEnabled = true;
    bool m_goalEnabled = true;
};
