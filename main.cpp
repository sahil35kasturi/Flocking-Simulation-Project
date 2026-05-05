#include "Simulation.h"

#define GL_SILENCE_DEPRECATION
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace {
constexpr int kWindowWidth = 1400;
constexpr int kWindowHeight = 900;
constexpr float kFixedDt = 1.0f / 120.0f;

struct AppState {
    Simulation simulation;
    bool drawDebug = false;
};

void setWindowTitle(GLFWwindow* window, const AppState& app) {
    const SimulationParameters& params = app.simulation.params();
    std::ostringstream stream;
    stream << "Flocking PBD | " << app.simulation.scenarioName()
           << " | birds=" << app.simulation.birds().size()
           << " sep=" << std::fixed << std::setprecision(1) << params.separationWeight
           << " align=" << params.alignmentWeight
           << " coh=" << params.cohesionWeight
           << " pred=" << params.predatorAvoidWeight
           << " speed=" << params.maxSpeed
           << " predators=" << app.simulation.predators().size()
           << (app.simulation.predator().enabled ? ":on" : ":off")
           << (app.simulation.goalEnabled() ? " | goal:on" : " | goal:off")
           << (app.simulation.isPaused() ? " | paused" : "");
    glfwSetWindowTitle(window, stream.str().c_str());
}

void printControls() {
    std::cout
        << "Controls\n"
        << "  Space: pause/resume\n"
        << "  R: reset simulation\n"
        << "  N/B: next or previous scenario\n"
        << "  P: toggle predator\n"
        << "  V: toggle goal-directed movement\n"
        << "  D: toggle debug overlays\n"
        << "  Up/Down: add or remove birds\n"
        << "  Q/A: separation weight +/-\n"
        << "  W/S: alignment weight +/-\n"
        << "  E/C: cohesion weight +/-\n"
        << "  T/G: predator avoidance weight +/-\n"
        << "  Y/H: max speed +/-\n"
        << std::endl;
}

void drawDisk(float radius, int segments) {
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(0.0f, 0.0f, 0.0f);
    for (int i = 0; i <= segments; ++i) {
        const float angle = static_cast<float>(i) / static_cast<float>(segments) * 2.0f * static_cast<float>(M_PI);
        glVertex3f(std::cos(angle) * radius, 0.0f, std::sin(angle) * radius);
    }
    glEnd();
}

void drawRing(float radius, int segments) {
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < segments; ++i) {
        const float angle = static_cast<float>(i) / static_cast<float>(segments) * 2.0f * static_cast<float>(M_PI);
        glVertex3f(std::cos(angle) * radius, 0.02f, std::sin(angle) * radius);
    }
    glEnd();
}

void drawBirdShape(const Eigen::Vector3f& position, const Eigen::Vector3f& heading, float scale, bool predator) {
    const Eigen::Vector3f dir = heading.normalized();
    const Eigen::Vector3f side(-dir.z(), 0.0f, dir.x());
    const Eigen::Vector3f nose = position + dir * scale * 1.6f;
    const Eigen::Vector3f left = position - dir * scale * 0.8f + side * scale * 0.7f;
    const Eigen::Vector3f right = position - dir * scale * 0.8f - side * scale * 0.7f;
    const Eigen::Vector3f tail = position - dir * scale * 1.1f;

    glBegin(GL_TRIANGLES);
    glVertex3f(nose.x(), 0.18f, nose.z());
    glVertex3f(left.x(), 0.04f, left.z());
    glVertex3f(right.x(), 0.04f, right.z());
    glEnd();

    glBegin(GL_TRIANGLES);
    glVertex3f(tail.x(), 0.05f, tail.z());
    glVertex3f(left.x(), 0.04f, left.z());
    glVertex3f(right.x(), 0.04f, right.z());
    glEnd();

    glColor3f(predator ? 0.35f : 0.04f, predator ? 0.04f : 0.10f, predator ? 0.03f : 0.13f);
    glBegin(GL_LINE_LOOP);
    glVertex3f(nose.x(), 0.19f, nose.z());
    glVertex3f(left.x(), 0.05f, left.z());
    glVertex3f(tail.x(), 0.06f, tail.z());
    glVertex3f(right.x(), 0.05f, right.z());
    glEnd();

    if (predator) {
        glBegin(GL_LINES);
        glVertex3f(position.x(), 0.1f, position.z());
        glVertex3f(nose.x(), 0.3f, nose.z());
        glEnd();
    }
}

void drawWorld(const AppState& app) {
    const float extent = app.simulation.params().worldHalfExtent;

    glColor3f(0.06f, 0.08f, 0.11f);
    glBegin(GL_QUADS);
    glVertex3f(-extent, -0.02f, -extent);
    glVertex3f(-extent, -0.02f, extent);
    glVertex3f(extent, -0.02f, extent);
    glVertex3f(extent, -0.02f, -extent);
    glEnd();

    glColor3f(0.18f, 0.24f, 0.30f);
    glBegin(GL_LINES);
    for (int i = static_cast<int>(-extent); i <= static_cast<int>(extent); i += 2) {
        glVertex3f(static_cast<float>(i), -0.01f, -extent);
        glVertex3f(static_cast<float>(i), -0.01f, extent);
        glVertex3f(-extent, -0.01f, static_cast<float>(i));
        glVertex3f(extent, -0.01f, static_cast<float>(i));
    }
    glEnd();

    glColor3f(0.55f, 0.68f, 0.78f);
    glBegin(GL_LINE_LOOP);
    glVertex3f(-extent, 0.0f, -extent);
    glVertex3f(-extent, 0.0f, extent);
    glVertex3f(extent, 0.0f, extent);
    glVertex3f(extent, 0.0f, -extent);
    glEnd();
}

void drawObstacles(const AppState& app) {
    for (const Obstacle& obstacle : app.simulation.obstacles()) {
        glPushMatrix();
        glTranslatef(obstacle.center.x(), 0.0f, obstacle.center.z());

        glColor3f(0.36f, 0.37f, 0.42f);
        drawDisk(obstacle.radius, 40);

        glColor3f(0.24f, 0.26f, 0.31f);
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < 40; ++i) {
            const float angle = static_cast<float>(i) / 40.0f * 2.0f * static_cast<float>(M_PI);
            glVertex3f(std::cos(angle) * obstacle.radius * 0.72f, 0.12f, std::sin(angle) * obstacle.radius * 0.72f);
        }
        glEnd();

        glColor3f(0.75f, 0.80f, 0.86f);
        drawRing(obstacle.radius, 40);

        if (app.drawDebug) {
            glColor4f(0.95f, 0.78f, 0.34f, 0.35f);
            drawRing(obstacle.radius + app.simulation.params().obstacleSenseRadius, 60);
        }
        glPopMatrix();
    }
}

void drawGoal(const AppState& app) {
    if (!app.simulation.goalEnabled()) {
        return;
    }

    const Eigen::Vector3f& goal = app.simulation.goal();
    glPushMatrix();
    glTranslatef(goal.x(), 0.0f, goal.z());

    glColor3f(0.25f, 0.95f, 0.55f);
    drawRing(1.3f, 48);

    glBegin(GL_LINES);
    glVertex3f(-1.8f, 0.05f, 0.0f);
    glVertex3f(1.8f, 0.05f, 0.0f);
    glVertex3f(0.0f, 0.05f, -1.8f);
    glVertex3f(0.0f, 0.05f, 1.8f);
    glEnd();

    glColor4f(0.25f, 0.95f, 0.55f, 0.18f);
    drawDisk(1.1f, 48);
    glPopMatrix();
}

void drawBirds(const AppState& app) {
    for (const Bird& bird : app.simulation.birds()) {
        const float speedRatio = std::clamp(bird.velocity.norm() / std::max(0.1f, app.simulation.params().maxSpeed), 0.0f, 1.0f);
        glColor3f(0.35f + 0.45f * speedRatio, 0.75f, 0.95f - 0.25f * speedRatio);
        drawBirdShape(bird.position, bird.heading, 0.45f, false);

        if (app.drawDebug) {
            glColor4f(0.3f, 0.7f, 0.9f, 0.25f);
            glBegin(GL_LINES);
            glVertex3f(bird.position.x(), 0.08f, bird.position.z());
            glVertex3f(bird.position.x() + bird.velocity.x() * 0.2f, 0.08f, bird.position.z() + bird.velocity.z() * 0.2f);
            glEnd();
        }
    }

    if (app.drawDebug) {
        const auto& birds = app.simulation.birds();
        for (std::size_t i = 0; i < birds.size(); i += 20) {
            glPushMatrix();
            glTranslatef(birds[i].position.x(), 0.0f, birds[i].position.z());
            glColor4f(0.25f, 0.65f, 0.95f, 0.22f);
            drawRing(app.simulation.params().neighborRadius, 42);
            glColor4f(0.9f, 0.95f, 1.0f, 0.28f);
            drawRing(app.simulation.params().separationRadius, 28);
            glPopMatrix();
        }
    }
}

void drawPredators(const AppState& app) {
    for (const Predator& predator : app.simulation.predators()) {
        if (!predator.enabled) {
            continue;
        }

        glColor3f(0.94f, 0.28f, 0.22f);
        drawBirdShape(predator.position, predator.heading, 0.9f, true);

        if (app.drawDebug) {
            glColor4f(0.98f, 0.4f, 0.3f, 0.35f);
            glPushMatrix();
            glTranslatef(predator.position.x(), 0.0f, predator.position.z());
            drawRing(app.simulation.params().predatorSenseRadius, 60);
            glColor4f(1.0f, 0.12f, 0.08f, 0.75f);
            drawRing(app.simulation.params().predatorCatchRadius, 28);
            glPopMatrix();
        }
    }
}

void render(const AppState& app, int width, int height) {
    const float aspect = static_cast<float>(width) / static_cast<float>(std::max(height, 1));

    glViewport(0, 0, width, height);
    glClearColor(0.015f, 0.02f, 0.03f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(48.0, aspect, 0.1, 200.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0.0, 24.0, 28.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    drawWorld(app);
    drawGoal(app);
    drawObstacles(app);
    drawBirds(app);
    drawPredators(app);
}

void adjustParam(float& value, float delta, float minValue, float maxValue) {
    value = std::clamp(value + delta, minValue, maxValue);
}

void keyCallback(GLFWwindow* window, int key, int, int action, int) {
    if (action != GLFW_PRESS && action != GLFW_REPEAT) {
        return;
    }

    auto* app = static_cast<AppState*>(glfwGetWindowUserPointer(window));
    SimulationParameters& params = app->simulation.params();

    switch (key) {
    case GLFW_KEY_ESCAPE:
        glfwSetWindowShouldClose(window, GLFW_TRUE);
        break;
    case GLFW_KEY_SPACE:
        app->simulation.setPaused(!app->simulation.isPaused());
        break;
    case GLFW_KEY_R:
        app->simulation.reset();
        break;
    case GLFW_KEY_N:
        app->simulation.nextScenario();
        break;
    case GLFW_KEY_B:
        app->simulation.previousScenario();
        break;
    case GLFW_KEY_P:
        app->simulation.togglePredator();
        break;
    case GLFW_KEY_V:
        app->simulation.toggleGoal();
        break;
    case GLFW_KEY_UP:
        app->simulation.resizeFlock(static_cast<int>(app->simulation.birds().size()) + 10);
        break;
    case GLFW_KEY_DOWN:
        app->simulation.resizeFlock(static_cast<int>(app->simulation.birds().size()) - 10);
        break;
    case GLFW_KEY_Q:
        adjustParam(params.separationWeight, 1.0f, 0.0f, 60.0f);
        break;
    case GLFW_KEY_A:
        adjustParam(params.separationWeight, -1.0f, 0.0f, 60.0f);
        break;
    case GLFW_KEY_W:
        adjustParam(params.alignmentWeight, 0.5f, 0.0f, 20.0f);
        break;
    case GLFW_KEY_S:
        adjustParam(params.alignmentWeight, -0.5f, 0.0f, 20.0f);
        break;
    case GLFW_KEY_E:
        adjustParam(params.cohesionWeight, 0.5f, 0.0f, 20.0f);
        break;
    case GLFW_KEY_C:
        adjustParam(params.cohesionWeight, -0.5f, 0.0f, 20.0f);
        break;
    case GLFW_KEY_D:
        app->drawDebug = !app->drawDebug;
        break;
    case GLFW_KEY_T:
        adjustParam(params.predatorAvoidWeight, 1.5f, 0.0f, 60.0f);
        break;
    case GLFW_KEY_G:
        adjustParam(params.predatorAvoidWeight, -1.5f, 0.0f, 60.0f);
        break;
    case GLFW_KEY_Y:
        adjustParam(params.maxSpeed, 0.5f, 1.0f, 20.0f);
        break;
    case GLFW_KEY_H:
        adjustParam(params.maxSpeed, -0.5f, 1.0f, 20.0f);
        break;
    default:
        break;
    }

    setWindowTitle(window, *app);
}
}

int main() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW." << std::endl;
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    GLFWwindow* window = glfwCreateWindow(kWindowWidth, kWindowHeight, "Flocking PBD", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window." << std::endl;
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    AppState app;
    glfwSetWindowUserPointer(window, &app);
    glfwSetKeyCallback(window, keyCallback);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    printControls();
    setWindowTitle(window, app);

    double previousTime = glfwGetTime();
    double accumulator = 0.0;

    while (!glfwWindowShouldClose(window)) {
        const double currentTime = glfwGetTime();
        double frameTime = currentTime - previousTime;
        previousTime = currentTime;
        frameTime = std::min(frameTime, 0.05);
        accumulator += frameTime;

        glfwPollEvents();

        while (accumulator >= kFixedDt) {
            app.simulation.update(kFixedDt);
            accumulator -= kFixedDt;
        }

        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(window, &width, &height);
        render(app, width, height);
        glfwSwapBuffers(window);
        setWindowTitle(window, app);
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
