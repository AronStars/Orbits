#include <cmath>
#include "Sim.h"

int main() {
    OrbitSimulator simulator;
    constexpr int screenWidth = 1600;
    constexpr int screenHeight = 900;
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(screenWidth, screenHeight, "Orbit Simulator");

    // Set up a three-body configuration.
    constexpr float mass = 10000.0f;
    constexpr float speed = 50.0f;
    constexpr Vector2 center = {static_cast<float>(screenWidth) / 2.0f, static_cast<float>(screenHeight) / 2.0f};

    constexpr float dx = 75.0f;
    constexpr float dy = 43.3f;

    Vector2 pos1 = {center.x + dx, center.y - dy};
    Vector2 vel1 = {pos1.y - center.y, -(pos1.x - center.x)}; {
        float const norm = sqrtf(vel1.x * vel1.x + vel1.y * vel1.y);
        vel1.x = (vel1.x / norm) * speed;
        vel1.y = (vel1.y / norm) * speed;
    }

    Vector2 pos2 = {center.x - dx, center.y - dy};
    Vector2 vel2 = {pos2.y - center.y, -(pos2.x - center.x)}; {
        float const norm = sqrtf(vel2.x * vel2.x + vel2.y * vel2.y);
        vel2.x = (vel2.x / norm) * speed;
        vel2.y = (vel2.y / norm) * speed;
    }

    Vector2 pos3 = {center.x, center.y + 2 * dy};
    Vector2 vel3 = {pos3.y - center.y, -(pos3.x - center.x)}; {
        float const norm = sqrtf(vel3.x * vel3.x + vel3.y * vel3.y);
        vel3.x = (vel3.x / norm) * speed;
        vel3.y = (vel3.y / norm) * speed;
    }
    simulator.addBody(pos1, vel1, mass, 8.f);
    simulator.addBody(pos2, vel2, mass, 8.f);
    simulator.addBody(pos3, vel3, mass, 8.f);
    SetTargetFPS(60);
    simulator.run();
    return 0;
}
