#include "Simulation.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace
{
    constexpr float kEpsilon = 1.0e-5f;
    constexpr float kPi = 3.14159265358979323846f;

    float randomRange(float minValue, float maxValue)
    {
        const float t = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
        return minValue + (maxValue - minValue) * t;
    }
}

Simulation::Simulation()
{
    std::srand(7);
    reset();
}

void Simulation::reset()
{
    configureScenario();
    m_birds.clear();
    m_birds.reserve(static_cast<std::size_t>(m_params.flockSize));

    for (int i = 0; i < m_params.flockSize; ++i)
    {
        Bird bird;
        bird.position = randomPointInWorld(6.0f);
        bird.predictedPosition = bird.position;
        bird.previousPosition = bird.position;
        bird.velocity = randomUnitVector() * randomRange(2.0f, m_params.maxSpeed * 0.65f);
        bird.heading = bird.velocity.squaredNorm() > kEpsilon ? bird.velocity.normalized() : Eigen::Vector3f::UnitX();
        m_birds.push_back(bird);
    }

    resetPredators();
}

void Simulation::resizeFlock(int newSize)
{
    m_params.flockSize = std::max(10, newSize);
    reset();
}

void Simulation::update(float dt)
{
    if (m_paused)
    {
        return;
    }

    updatePredators(dt);
    removeCaughtBirds();
    integrateBirds(dt);
    projectConstraints();
    finalizeBirds(dt);
}

void Simulation::setPaused(bool paused)
{
    m_paused = paused;
}

bool Simulation::isPaused() const
{
    return m_paused;
}

void Simulation::togglePredator()
{
    m_predatorsEnabled = !m_predatorsEnabled;
    for (Predator &predator : m_predators)
    {
        predator.enabled = m_predatorsEnabled;
    }
}

void Simulation::setPredatorEnabled(bool enabled)
{
    m_predatorsEnabled = enabled;
    for (Predator &predator : m_predators)
    {
        predator.enabled = enabled;
    }
}

void Simulation::toggleGoal()
{
    m_goalEnabled = !m_goalEnabled;
}

void Simulation::nextScenario()
{
    const int next = (static_cast<int>(m_scenario) + 1) % 4;
    setScenario(static_cast<Scenario>(next));
}

void Simulation::previousScenario()
{
    const int current = static_cast<int>(m_scenario);
    const int next = (current + 3) % 4;
    setScenario(static_cast<Scenario>(next));
}

void Simulation::setScenario(Scenario scenario)
{
    m_scenario = scenario;
    reset();
}

SimulationParameters &Simulation::params()
{
    return m_params;
}

const SimulationParameters &Simulation::params() const
{
    return m_params;
}

const std::vector<Bird> &Simulation::birds() const
{
    return m_birds;
}

const std::vector<Obstacle> &Simulation::obstacles() const
{
    return m_obstacles;
}

const Predator &Simulation::predator() const
{
    return m_predators.front();
}

const std::vector<Predator> &Simulation::predators() const
{
    return m_predators;
}

const Eigen::Vector3f &Simulation::goal() const
{
    return m_goal;
}

bool Simulation::goalEnabled() const
{
    return m_goalEnabled;
}

Scenario Simulation::scenario() const
{
    return m_scenario;
}

const char *Simulation::scenarioName() const
{
    switch (m_scenario)
    {
    case Scenario::OpenField:
        return "Open Field";
    case Scenario::ObstacleCourse:
        return "Obstacle Course";
    case Scenario::SplitAndRegroup:
        return "Split And Regroup";
    case Scenario::DenseObstacles:
        return "Dense Obstacles";
    default:
        return "Unknown";
    }
}

void Simulation::configureScenario()
{
    switch (m_scenario)
    {
    case Scenario::OpenField:
        m_goal = Eigen::Vector3f(15.0f, 0.0f, 15.0f);
        m_obstacles = {
            {Eigen::Vector3f(-5.0f, 0.0f, -2.5f), 2.4f},
            {Eigen::Vector3f(4.0f, 0.0f, 5.5f), 3.0f},
            {Eigen::Vector3f(8.0f, 0.0f, -7.0f), 2.0f},
        };
        break;
    case Scenario::ObstacleCourse:
        m_goal = Eigen::Vector3f(17.0f, 0.0f, 16.0f);
        m_obstacles = {
            {Eigen::Vector3f(-12.0f, 0.0f, -7.0f), 2.3f},
            {Eigen::Vector3f(-5.0f, 0.0f, -7.0f), 2.0f},
            {Eigen::Vector3f(3.0f, 0.0f, -7.0f), 2.3f},
            {Eigen::Vector3f(11.0f, 0.0f, -7.0f), 2.0f},
            {Eigen::Vector3f(-9.0f, 0.0f, 2.0f), 2.5f},
            {Eigen::Vector3f(-1.0f, 0.0f, 2.0f), 2.2f},
            {Eigen::Vector3f(7.5f, 0.0f, 2.0f), 2.5f},
            {Eigen::Vector3f(-13.0f, 0.0f, 10.0f), 2.0f},
            {Eigen::Vector3f(-4.5f, 0.0f, 10.0f), 2.4f},
            {Eigen::Vector3f(5.0f, 0.0f, 10.0f), 2.0f},
            {Eigen::Vector3f(13.0f, 0.0f, 10.0f), 2.4f},
        };
        break;
    case Scenario::SplitAndRegroup:
        m_goal = Eigen::Vector3f(0.0f, 0.0f, 17.0f);
        m_obstacles = {
            {Eigen::Vector3f(-3.0f, 0.0f, -1.0f), 3.2f},
            {Eigen::Vector3f(3.0f, 0.0f, -1.0f), 3.2f},
            {Eigen::Vector3f(0.0f, 0.0f, 6.0f), 2.6f},
            {Eigen::Vector3f(-10.5f, 0.0f, 7.0f), 2.0f},
            {Eigen::Vector3f(10.5f, 0.0f, 7.0f), 2.0f},
        };
        break;
    case Scenario::DenseObstacles:
        m_goal = Eigen::Vector3f(-16.0f, 0.0f, 15.0f);
        m_obstacles = {
            {Eigen::Vector3f(-14.0f, 0.0f, -11.0f), 1.8f},
            {Eigen::Vector3f(-8.0f, 0.0f, -13.0f), 2.2f},
            {Eigen::Vector3f(-1.0f, 0.0f, -10.0f), 2.0f},
            {Eigen::Vector3f(6.0f, 0.0f, -13.0f), 1.8f},
            {Eigen::Vector3f(13.0f, 0.0f, -10.0f), 2.4f},
            {Eigen::Vector3f(-13.0f, 0.0f, -2.0f), 2.2f},
            {Eigen::Vector3f(-5.0f, 0.0f, -1.0f), 1.9f},
            {Eigen::Vector3f(2.5f, 0.0f, -3.0f), 2.4f},
            {Eigen::Vector3f(10.5f, 0.0f, -1.0f), 2.0f},
            {Eigen::Vector3f(-10.0f, 0.0f, 7.0f), 2.4f},
            {Eigen::Vector3f(-2.0f, 0.0f, 8.0f), 1.8f},
            {Eigen::Vector3f(6.0f, 0.0f, 7.0f), 2.1f},
            {Eigen::Vector3f(14.0f, 0.0f, 9.0f), 1.9f},
        };
        break;
    }
}

void Simulation::resetPredators()
{
    m_predators.clear();
    const float extent = m_params.worldHalfExtent * 0.75f;

    auto addPredator = [&](const Eigen::Vector3f &position, const Eigen::Vector3f &velocity) {
        Predator predator;
        predator.position = position;
        predator.velocity = velocity;
        predator.heading = velocity.squaredNorm() > kEpsilon ? velocity.normalized() : Eigen::Vector3f::UnitX();
        predator.enabled = m_predatorsEnabled;
        m_predators.push_back(predator);
    };

    switch (m_scenario)
    {
    case Scenario::OpenField:
        addPredator(Eigen::Vector3f(0.0f, 0.0f, -extent), Eigen::Vector3f(2.5f, 0.0f, 2.0f));
        break;
    case Scenario::ObstacleCourse:
        addPredator(Eigen::Vector3f(-extent, 0.0f, -extent * 0.5f), Eigen::Vector3f(3.5f, 0.0f, 2.2f));
        break;
    case Scenario::SplitAndRegroup:
        addPredator(Eigen::Vector3f(-extent, 0.0f, -extent * 0.35f), Eigen::Vector3f(3.0f, 0.0f, 2.0f));
        addPredator(Eigen::Vector3f(extent, 0.0f, -extent * 0.25f), Eigen::Vector3f(-3.0f, 0.0f, 2.0f));
        break;
    case Scenario::DenseObstacles:
        addPredator(Eigen::Vector3f(extent * 0.8f, 0.0f, -extent), Eigen::Vector3f(-2.0f, 0.0f, 3.0f));
        addPredator(Eigen::Vector3f(-extent * 0.8f, 0.0f, extent * 0.8f), Eigen::Vector3f(2.0f, 0.0f, -2.5f));
        break;
    }
}

Eigen::Vector3f Simulation::randomPointInWorld(float margin) const
{
    const float extent = std::max(2.0f, m_params.worldHalfExtent - margin);
    return Eigen::Vector3f(
        randomRange(-extent, extent),
        0.0f,
        randomRange(-extent, extent));
}

Eigen::Vector3f Simulation::randomUnitVector() const
{
    const float angle = randomRange(0.0f, 2.0f * kPi);
    return Eigen::Vector3f(std::cos(angle), 0.0f, std::sin(angle));
}

Eigen::Vector3f Simulation::clampMagnitude(const Eigen::Vector3f &value, float maxMagnitude) const
{
    const float norm = value.norm();
    if (norm <= maxMagnitude || norm < kEpsilon)
    {
        return value;
    }
    return value * (maxMagnitude / norm);
}

Eigen::Vector3f Simulation::computeBirdAcceleration(std::size_t index) const
{
    const Bird &bird = m_birds[index];
    Eigen::Vector3f separation = Eigen::Vector3f::Zero();
    Eigen::Vector3f alignment = Eigen::Vector3f::Zero();
    Eigen::Vector3f cohesionCenter = Eigen::Vector3f::Zero();
    int alignmentCount = 0;
    int cohesionCount = 0;

    for (std::size_t otherIndex = 0; otherIndex < m_birds.size(); ++otherIndex)
    {
        if (otherIndex == index)
        {
            continue;
        }

        const Bird &other = m_birds[otherIndex];
        const Eigen::Vector3f offset = bird.position - other.position;
        const float distance = offset.norm();
        if (distance < kEpsilon)
        {
            continue;
        }

        if (distance < m_params.separationRadius)
        {
            separation += offset.normalized() * ((m_params.separationRadius - distance) / m_params.separationRadius);
        }

        if (distance < m_params.neighborRadius)
        {
            alignment += other.velocity;
            cohesionCenter += other.position;
            ++alignmentCount;
            ++cohesionCount;
        }
    }

    Eigen::Vector3f force = Eigen::Vector3f::Zero();

    if (separation.squaredNorm() > kEpsilon)
    {
        force += separation.normalized() * m_params.separationWeight;
    }

    if (alignmentCount > 0)
    {
        alignment /= static_cast<float>(alignmentCount);
        if (alignment.squaredNorm() > kEpsilon)
        {
            const Eigen::Vector3f desired = alignment.normalized() * m_params.maxSpeed;
            force += (desired - bird.velocity) * m_params.alignmentWeight;
        }
    }

    if (cohesionCount > 0)
    {
        cohesionCenter /= static_cast<float>(cohesionCount);
        Eigen::Vector3f toCenter = cohesionCenter - bird.position;
        if (toCenter.squaredNorm() > kEpsilon)
        {
            force += toCenter.normalized() * m_params.cohesionWeight;
        }
    }

    for (const Obstacle &obstacle : m_obstacles)
    {
        Eigen::Vector3f away = bird.position - obstacle.center;
        float distance = away.norm();
        if (distance < kEpsilon)
        {
            away = randomUnitVector();
            distance = 1.0f;
        }

        const float clearance = distance - obstacle.radius;
        if (clearance < m_params.obstacleSenseRadius)
        {
            const float safeClearance = std::max(clearance, 0.1f);
            force += away.normalized() * (m_params.obstacleWeight / safeClearance);
        }
    }

    if (m_predatorsEnabled)
    {
        for (const Predator &predator : m_predators)
        {
            if (!predator.enabled)
            {
                continue;
            }

            const Eigen::Vector3f away = bird.position - predator.position;
            const float distance = away.norm();
            if (distance < m_params.predatorSenseRadius && distance > kEpsilon)
            {
                const float urgency = (m_params.predatorSenseRadius - distance) / m_params.predatorSenseRadius;
                force += away.normalized() * (m_params.predatorAvoidWeight * urgency);
            }
        }
    }

    if (m_goalEnabled)
    {
        const Eigen::Vector3f toGoal = m_goal - bird.position;
        const float distance = toGoal.norm();
        if (distance > 1.25f)
        {
            const float speedScale = std::clamp(distance / 9.0f, 0.25f, 1.0f);
            const Eigen::Vector3f desired = toGoal.normalized() * (m_params.maxSpeed * speedScale);
            force += (desired - bird.velocity) * m_params.goalWeight;
        }
        else
        {
            force += -bird.velocity * (m_params.goalWeight * 0.6f);
        }
    }

    force += computeBoundaryForce(bird.position);
    return clampMagnitude(force, m_params.maxAcceleration);
}

Eigen::Vector3f Simulation::computeBoundaryForce(const Eigen::Vector3f &position) const
{
    Eigen::Vector3f force = Eigen::Vector3f::Zero();
    const float softLimit = m_params.worldHalfExtent - 3.5f;

    if (position.x() > softLimit)
    {
        force.x() -= (position.x() - softLimit) * m_params.boundaryWeight;
    }
    else if (position.x() < -softLimit)
    {
        force.x() += (-softLimit - position.x()) * m_params.boundaryWeight;
    }

    if (position.z() > softLimit)
    {
        force.z() -= (position.z() - softLimit) * m_params.boundaryWeight;
    }
    else if (position.z() < -softLimit)
    {
        force.z() += (-softLimit - position.z()) * m_params.boundaryWeight;
    }

    return force;
}

void Simulation::integrateBirds(float dt)
{
    for (std::size_t i = 0; i < m_birds.size(); ++i)
    {
        Bird &bird = m_birds[i];
        bird.previousPosition = bird.position;

        const Eigen::Vector3f acceleration = computeBirdAcceleration(i);
        bird.velocity += acceleration * dt;
        bird.velocity = clampMagnitude(bird.velocity, m_params.maxSpeed);
        bird.velocity *= m_params.damping;
        bird.predictedPosition = bird.position + bird.velocity * dt;
        bird.predictedPosition.y() = 0.0f;
    }
}

void Simulation::projectConstraints()
{
    for (int iteration = 0; iteration < m_params.solverIterations; ++iteration)
    {
        solveBirdSeparation();
        solveObstacleCollisions();
        solveBoundaryCollisions();
    }
}

void Simulation::solveBirdSeparation()
{
    const float minDistance = m_params.birdRadius * 2.0f;

    for (std::size_t i = 0; i < m_birds.size(); ++i)
    {
        for (std::size_t j = i + 1; j < m_birds.size(); ++j)
        {
            Eigen::Vector3f delta = m_birds[j].predictedPosition - m_birds[i].predictedPosition;
            float distance = delta.norm();
            if (distance < kEpsilon)
            {
                delta = randomUnitVector() * 0.01f;
                distance = delta.norm();
            }

            if (distance < minDistance)
            {
                const Eigen::Vector3f correction = delta.normalized() * ((minDistance - distance) * 0.5f);
                m_birds[i].predictedPosition -= correction;
                m_birds[j].predictedPosition += correction;
            }
        }
    }
}

void Simulation::solveObstacleCollisions()
{
    const float padding = m_params.birdRadius + 0.15f;

    for (Bird &bird : m_birds)
    {
        for (const Obstacle &obstacle : m_obstacles)
        {
            Eigen::Vector3f delta = bird.predictedPosition - obstacle.center;
            float distance = delta.norm();
            const float minDistance = obstacle.radius + padding;

            if (distance < kEpsilon)
            {
                delta = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
                distance = 1.0f;
            }

            if (distance < minDistance)
            {
                bird.predictedPosition = obstacle.center + delta.normalized() * minDistance;
                bird.predictedPosition.y() = 0.0f;
            }
        }
    }

    if (!m_predatorsEnabled)
    {
        return;
    }

    for (Predator &predator : m_predators)
    {
        if (!predator.enabled)
        {
            continue;
        }

        for (const Obstacle &obstacle : m_obstacles)
        {
            Eigen::Vector3f delta = predator.position - obstacle.center;
            float distance = delta.norm();
            const float minDistance = obstacle.radius + 0.35f;

            if (distance < minDistance)
            {
                if (distance < kEpsilon)
                {
                    delta = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
                    distance = 1.0f;
                }
                predator.position = obstacle.center + delta.normalized() * minDistance;
                predator.position.y() = 0.0f;
            }
        }
    }
}

void Simulation::solveBoundaryCollisions()
{
    const float extent = m_params.worldHalfExtent - m_params.birdRadius;

    for (Bird &bird : m_birds)
    {
        bird.predictedPosition.x() = std::clamp(bird.predictedPosition.x(), -extent, extent);
        bird.predictedPosition.z() = std::clamp(bird.predictedPosition.z(), -extent, extent);
        bird.predictedPosition.y() = 0.0f;
    }

    if (!m_predatorsEnabled)
    {
        return;
    }

    for (Predator &predator : m_predators)
    {
        if (!predator.enabled)
        {
            continue;
        }

        predator.position.x() = std::clamp(predator.position.x(), -extent, extent);
        predator.position.z() = std::clamp(predator.position.z(), -extent, extent);
        predator.position.y() = 0.0f;
    }
}

void Simulation::finalizeBirds(float dt)
{
    for (Bird &bird : m_birds)
    {
        bird.velocity = (bird.predictedPosition - bird.position) / std::max(dt, 1.0e-4f);
        bird.velocity = clampMagnitude(bird.velocity, m_params.maxSpeed);
        bird.position = bird.predictedPosition;

        if (bird.velocity.squaredNorm() > kEpsilon)
        {
            const Eigen::Vector3f desired = bird.velocity.normalized();
            bird.heading = (bird.heading * 0.85f + desired * 0.15f).normalized();
        }
    }
}

void Simulation::updatePredators(float dt)
{
    if (!m_predatorsEnabled || m_birds.empty())
    {
        return;
    }

    Eigen::Vector3f flockCenter = Eigen::Vector3f::Zero();
    for (const Bird &bird : m_birds)
    {
        flockCenter += bird.position;
    }
    flockCenter /= static_cast<float>(m_birds.size());

    for (std::size_t predatorIndex = 0; predatorIndex < m_predators.size(); ++predatorIndex)
    {
        Predator &predator = m_predators[predatorIndex];
        if (!predator.enabled)
        {
            continue;
        }

        std::size_t closestIndex = 0;
        float closestDistance = std::numeric_limits<float>::max();
        for (std::size_t i = 0; i < m_birds.size(); ++i)
        {
            const float distance = (m_birds[i].position - predator.position).norm();
            if (distance < closestDistance)
            {
                closestDistance = distance;
                closestIndex = i;
            }
        }

        const float side = predatorIndex % 2 == 0 ? 1.0f : -1.0f;
        const Eigen::Vector3f targetOffset(side * 1.2f, 0.0f, 0.6f * static_cast<float>(predatorIndex));
        const Eigen::Vector3f target = m_birds[closestIndex].position * 0.72f + flockCenter * 0.28f + targetOffset;
        Eigen::Vector3f desired = target - predator.position;
        if (desired.squaredNorm() > kEpsilon)
        {
            desired.normalize();
            desired *= m_params.predatorMaxSpeed;
        }

        Eigen::Vector3f steering = (desired - predator.velocity) * m_params.predatorCatchupWeight;
        steering += computeBoundaryForce(predator.position) * 0.75f;

        for (const Obstacle &obstacle : m_obstacles)
        {
            Eigen::Vector3f away = predator.position - obstacle.center;
            float distance = away.norm();
            if (distance < kEpsilon)
            {
                away = randomUnitVector();
                distance = 1.0f;
            }

            const float clearance = distance - obstacle.radius;
            if (clearance < m_params.obstacleSenseRadius * 0.65f)
            {
                const float safeClearance = std::max(clearance, 0.2f);
                steering += away.normalized() * (m_params.obstacleWeight * 0.6f / safeClearance);
            }
        }

        for (std::size_t otherIndex = 0; otherIndex < m_predators.size(); ++otherIndex)
        {
            if (otherIndex == predatorIndex || !m_predators[otherIndex].enabled)
            {
                continue;
            }
            const Eigen::Vector3f away = predator.position - m_predators[otherIndex].position;
            const float distance = away.norm();
            if (distance < 3.0f && distance > kEpsilon)
            {
                steering += away.normalized() * (3.0f - distance) * 4.0f;
            }
        }

        steering = clampMagnitude(steering, m_params.predatorAcceleration);

        predator.velocity += steering * dt;
        predator.velocity = clampMagnitude(predator.velocity, m_params.predatorMaxSpeed);
        predator.position += predator.velocity * dt;
        predator.position.y() = 0.0f;

        if (predator.velocity.squaredNorm() > kEpsilon)
        {
            const Eigen::Vector3f desiredHeading = predator.velocity.normalized();
            predator.heading = (predator.heading * 0.8f + desiredHeading * 0.2f).normalized();
        }
    }
}

void Simulation::removeCaughtBirds()
{
    if (!m_predatorsEnabled || m_birds.empty())
    {
        return;
    }

    const float catchRadiusSq = m_params.predatorCatchRadius * m_params.predatorCatchRadius;
    m_birds.erase(
        std::remove_if(
            m_birds.begin(),
            m_birds.end(),
            [&](const Bird &bird) {
                for (const Predator &predator : m_predators)
                {
                    if (!predator.enabled)
                    {
                        continue;
                    }

                    if ((bird.position - predator.position).squaredNorm() <= catchRadiusSq)
                    {
                        return true;
                    }
                }
                return false;
            }),
        m_birds.end());
}
