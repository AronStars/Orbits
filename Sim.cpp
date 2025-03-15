#include "Sim.h"
#include <cmath>
#include <cstdio>
#include <algorithm> // For std::min and std::max

OrbitSimulator::OrbitSimulator(): dispposx(0), dispposy(0), dispvelx(0), dispvely(0), selectedIndex(-1),
                                  isDragging(false),
                                  G(50.0F), dtScale(1.0f){
}

// Add a new body to the simulator.
void OrbitSimulator::addBody(const Vector2 &position, const Vector2 &velocity, float mass, float radius) {
    bodies.emplace_back(position, velocity, mass, radius);
}

OrbitSimulator::~OrbitSimulator() {
    CloseWindow();
}

float OrbitSimulator::distance(Vector2 a, Vector2 b) {
    return sqrtf((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
}

int OrbitSimulator::findBodyAtPosition(Vector2 pos) const {
    for (size_t i = 0; i < bodies.size(); i++) {
        if (distance(bodies[i].position, pos) <= bodies[i].radius)
            return static_cast<int>(i);
    }
    return -1;
}

void OrbitSimulator::handleInput(float const dt) {
    Vector2 const mousePos = GetMousePosition();

    if (isDragging && selectedIndex >= 0 && selectedIndex < static_cast<int>(bodies.size())) {
        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
            bodies[selectedIndex].position = mousePos;
            bodies[selectedIndex].velocity = { 0.f, 0.f };
        }
    }

    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        int index = findBodyAtPosition(mousePos);
        if (index != -1) {
            selectedIndex = index;
            isDragging = true;
        } else {
            bodies.push_back(Body(mousePos, { 0.f, 0.f }, 100.f, 8.f));
            selectedIndex = static_cast<int>(bodies.size()) - 1;
            isDragging = true;
        }
    }

    if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
        isDragging = false;
    }

    if (!isDragging && selectedIndex >= 0 && selectedIndex < static_cast<int>(bodies.size())) {
        constexpr float velStep = 10.0f;
        if (IsKeyDown(KEY_RIGHT))
            bodies[selectedIndex].velocity.x += velStep * dt;
        if (IsKeyDown(KEY_LEFT))
            bodies[selectedIndex].velocity.x -= velStep * dt;
        if (IsKeyDown(KEY_UP))
            bodies[selectedIndex].velocity.y -= velStep * dt;
        if (IsKeyDown(KEY_DOWN))
            bodies[selectedIndex].velocity.y += velStep * dt;
        if (IsKeyDown(KEY_S))
            bodies[selectedIndex].mass = std::max(1.0f, bodies[selectedIndex].mass * 0.99f);
        if (IsKeyDown(KEY_W))
            bodies[selectedIndex].mass *= 1.01f;
        if (IsKeyDown(KEY_A))
            bodies[selectedIndex].radius = std::max(1.0f, bodies[selectedIndex].radius - 1.f);
        if (IsKeyDown(KEY_D))
            bodies[selectedIndex].radius += 1.f;

    }

    if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) {
        int index = findBodyAtPosition(mousePos);
        if (index != -1) {
            if (selectedIndex == index)
                selectedIndex = -1;
            bodies.erase(bodies.begin() + index);
        }
    }
}

// Calculate the acceleration of body i due to all other bodies.
Vector2 OrbitSimulator::calculateAcceleration(int i, const std::vector<Body>& currentBodies) const {
    Vector2 acceleration = {0.f, 0.f};
    for (size_t j = 0; j < currentBodies.size(); j++) {
        if (i == j) continue;
        Vector2 const direction = { currentBodies[j].position.x - currentBodies[i].position.x,
                              currentBodies[j].position.y - currentBodies[i].position.y };
        float distSq = direction.x * direction.x + direction.y * direction.y;

        // Softening factor to prevent extremely high accelerations at close distances.
        // This also helps with numerical stability.  Choose a value related to the typical
        // body radius or a bit smaller.
        constexpr float softening = 10.0f;
        distSq += softening * softening;  // Add softening to the squared distance

        float const dist = sqrtf(distSq);
        Vector2 const normDir = { direction.x / dist, direction.y / dist };
        float const accelMag = G * currentBodies[j].mass / distSq;
        acceleration.x += normDir.x * accelMag;
        acceleration.y += normDir.y * accelMag;
    }
    return acceleration;
}

void OrbitSimulator::handleCollisions() {
    for (size_t i = 0; i < bodies.size(); i++) {
        for (size_t j = i + 1; j < bodies.size(); j++) {
            float const dist = distance(bodies[i].position, bodies[j].position);
            float const min_dist = bodies[i].radius + bodies[j].radius;

            if (dist < min_dist) {
                // Collision detected!  Implement elastic collision response.

                // 1. Calculate the normal vector (direction of collision).
                Vector2 collisionNormal = {
                    bodies[j].position.x - bodies[i].position.x,
                    bodies[j].position.y - bodies[i].position.y
                };
                float const norm = sqrtf(collisionNormal.x * collisionNormal.x + collisionNormal.y * collisionNormal.y);
                collisionNormal.x /= norm;  // Normalize
                collisionNormal.y /= norm;

                // 2. Calculate the relative velocity.
                Vector2 const relativeVelocity = {
                    bodies[j].velocity.x - bodies[i].velocity.x,
                    bodies[j].velocity.y - bodies[i].velocity.y
                };

                // 3. Calculate the velocity component along the normal.
                float const velocityAlongNormal = relativeVelocity.x * collisionNormal.x + relativeVelocity.y * collisionNormal.y;

                // 4. If the bodies are moving apart, do nothing (prevents sticking).
                if (velocityAlongNormal > 0) continue;

                // 5. Calculate the impulse scalar (using the restitution coefficient).
                //    Restitution = 1.0 for perfectly elastic collisions.
                constexpr float restitution = 0.95f;
                float impulseScalar = -(1 + restitution) * velocityAlongNormal;
                impulseScalar /= (1 / bodies[i].mass + 1 / bodies[j].mass);

                // 6. Apply the impulse to each body.
                Vector2 const impulse = {
                    impulseScalar * collisionNormal.x,
                    impulseScalar * collisionNormal.y
                };

                bodies[i].velocity.x -= impulse.x / bodies[i].mass;
                bodies[i].velocity.y -= impulse.y / bodies[i].mass;
                bodies[j].velocity.x += impulse.x / bodies[j].mass;
                bodies[j].velocity.y += impulse.y / bodies[j].mass;

                // 7. Positional correction to prevent overlap (penetration resolution).
                float const penetration = min_dist - dist;
                constexpr float percent = 0.2f; // Percentage of penetration to correct per step
                constexpr float slop = 0.01f;    // Minimum penetration to correct
                Vector2 const correction = {
                    (std::max(penetration - slop, 0.0f) / (1 / bodies[i].mass + 1/bodies[j].mass)) * percent * collisionNormal.x,
                    (std::max(penetration - slop, 0.0f) / (1 / bodies[i].mass + 1/bodies[j].mass)) * percent * collisionNormal.y
                };
                bodies[i].position.x -= correction.x / bodies[i].mass;
                bodies[i].position.y -= correction.y / bodies[i].mass;
                bodies[j].position.x += correction.x / bodies[j].mass;
                bodies[j].position.y += correction.y / bodies[j].mass;

            }
        }
    }
}

void OrbitSimulator::updatePhysics(float dt) {
    // Velocity Verlet integration

    // 1. Calculate accelerations at the initial positions.
    std::vector<Vector2> accelerations(bodies.size());
    for (int i = 0; i < static_cast<int>(bodies.size()); i++) {
        if (!(isDragging && i == selectedIndex)) {
             accelerations[i] = calculateAcceleration(i, bodies);
        }
    }

    // 2. Update positions using current velocities and accelerations.
    for (size_t i = 0; i < bodies.size(); i++) {
        if (!(isDragging && static_cast<int>(i) == selectedIndex)) {
            bodies[i].position.x += bodies[i].velocity.x * dt + 0.5f * accelerations[i].x * dt * dt;
            bodies[i].position.y += bodies[i].velocity.y * dt + 0.5f * accelerations[i].y * dt * dt;
        }
    }

    // 3. Calculate the *new* accelerations at the updated positions.
    std::vector<Vector2> newAccelerations(bodies.size());
    for (int i = 0; i < static_cast<int>(bodies.size()); i++) {
        if (!(isDragging && i == selectedIndex)) {
            newAccelerations[i] = calculateAcceleration(i, bodies);
        }
    }

    // 4. Update velocities using the average of the old and new accelerations.
    for (size_t i = 0; i < bodies.size(); i++) {
        if (!(isDragging && static_cast<int>(i) == selectedIndex)) {
            bodies[i].velocity.x += 0.5f * (accelerations[i].x + newAccelerations[i].x) * dt;
            bodies[i].velocity.y += 0.5f * (accelerations[i].y + newAccelerations[i].y) * dt;
        }
    }

    // Handle collisions *after* the physics update.
    handleCollisions();
}

float predictionSteps = 300.0f;

std::vector<std::vector<Vector2> > OrbitSimulator::computePredictedPaths() const {
    if (IsKeyDown(KEY_I))
        predictionSteps *= 1.1f;
    if (IsKeyDown(KEY_K))
        predictionSteps *= 0.9f;
    std::vector<Body> predBodies = bodies; // Copy current state
    std::vector<std::vector<Vector2> > predictedPaths(bodies.size());



    for (int step = 0; static_cast<float>(step) < predictionSteps; step++) {
        constexpr float dtPred = 0.01f;
        // Velocity Verlet integration (within the prediction loop)
        std::vector<Vector2> accelerations(predBodies.size());
        for (int i = 0; i < static_cast<int>(predBodies.size()); i++) {
            accelerations[i] = calculateAcceleration((i), predBodies);
        }

        for (size_t i = 0; i < predBodies.size(); i++) {
            // Use a fixed time step for prediction.  This doesn't have to be the same as the
            // main simulation timestep, and often a smaller value is better for smoother paths.

            predBodies[i].position.x += predBodies[i].velocity.x * dtPred + 0.5f * accelerations[i].x * dtPred * dtPred;
            predBodies[i].position.y += predBodies[i].velocity.y * dtPred + 0.5f * accelerations[i].y * dtPred * dtPred;
        }

        std::vector<Vector2> newAccelerations(predBodies.size());
        for (int i = 0; i < static_cast<int>(predBodies.size()); i++) {
            newAccelerations[i] = calculateAcceleration((i), predBodies);
        }

        for (size_t i = 0; i < predBodies.size(); i++) {
            predBodies[i].velocity.x += 0.5f * (accelerations[i].x + newAccelerations[i].x) * dtPred;
            predBodies[i].velocity.y += 0.5f * (accelerations[i].y + newAccelerations[i].y) * dtPred;
            predictedPaths[i].push_back(predBodies[i].position);
        }

        // Handle collisions within predicted paths
        for (size_t i = 0; i < predBodies.size(); i++) {
            for (size_t j = i + 1; j < predBodies.size(); j++) {
                float const dist = distance(predBodies[i].position, predBodies[j].position);

                if (float const min_dist = predBodies[i].radius + predBodies[j].radius; dist < min_dist) {
                    // Collision detected! Elastic collision response.

                    // 1. Calculate the normal vector (direction of collision).
                    Vector2 collisionNormal = {
                        predBodies[j].position.x - predBodies[i].position.x,
                        predBodies[j].position.y - predBodies[i].position.y
                    };
                    float const norm = sqrtf(collisionNormal.x * collisionNormal.x + collisionNormal.y * collisionNormal.y);
                    collisionNormal.x /= norm; // Normalize
                    collisionNormal.y /= norm;

                    // 2. Calculate the relative velocity.
                    Vector2 const relativeVelocity = {
                        predBodies[j].velocity.x - predBodies[i].velocity.x,
                        predBodies[j].velocity.y - predBodies[i].velocity.y
                    };

                    // 3. Calculate the velocity component along the normal.
                    float const velocityAlongNormal = relativeVelocity.x * collisionNormal.x + relativeVelocity.y *
                                                collisionNormal.y;

                    // 4. If the bodies are moving apart, skip.
                    if (velocityAlongNormal > 0) continue;

                    // 5. Calculate the impulse scalar (using restitution).
                    constexpr float restitution = 0.95f;
                    float impulseScalar = -(1 + restitution) * velocityAlongNormal;
                    impulseScalar /= (1 / predBodies[i].mass + 1 / predBodies[j].mass);

                    // 6. Apply the impulse to each body.
                    Vector2 impulse = {
                        impulseScalar * collisionNormal.x,
                        impulseScalar * collisionNormal.y
                    };

                    predBodies[i].velocity.x -= impulse.x / predBodies[i].mass;
                    predBodies[i].velocity.y -= impulse.y / predBodies[i].mass;
                    predBodies[j].velocity.x += impulse.x / predBodies[j].mass;
                    predBodies[j].velocity.y += impulse.y / predBodies[j].mass;

                    // 7. Positional correction to prevent overlap.
                    float const penetration = min_dist - dist;
                    constexpr float percent = 0.2f; // Percentage of penetration to correct per step
                    constexpr float slop = 0.01f; // Minimum penetration to correct
                    Vector2 correction = {
                        (std::max(penetration - slop, 0.0f) / (1 / predBodies[i].mass + 1 / predBodies[j].mass)) *
                        percent * collisionNormal.x,
                        (std::max(penetration - slop, 0.0f) / (1 / predBodies[i].mass + 1 / predBodies[j].mass)) *
                        percent * collisionNormal.y
                    };
                    predBodies[i].position.x -= correction.x / predBodies[i].mass;
                    predBodies[i].position.y -= correction.y / predBodies[i].mass;
                    predBodies[j].position.x += correction.x / predBodies[j].mass;
                    predBodies[j].position.y += correction.y / predBodies[j].mass;
                }
            }
        }
    }
    return predictedPaths;
}

void OrbitSimulator::draw() {
    BeginDrawing();
    ClearBackground(BLACK);

    // Draw predicted paths.
    std::vector<std::vector<Vector2>> predictedPaths = computePredictedPaths();
    for (const auto &path: predictedPaths) {
        if (!path.empty() && path.size() > 1) {
            DrawLineStrip(path.data(),
                          static_cast<int>(path.size()),
                          YELLOW);
        }
    }

    // Draw bodies.
    for (size_t i = 0; i < bodies.size(); i++) {
        const Color col = (static_cast<int>(i) == selectedIndex) ? RED : WHITE;
        DrawCircleV(bodies[i].position, bodies[i].radius, col);
    }

    // Render instructions.
    DrawText("Left click & drag: Pick up or create a body", 10, 10, 15, LIGHTGRAY);
    DrawText("Right click on a body: Remove it", 10, 40, 15, LIGHTGRAY);
    DrawText("Arrow keys (when not dragging): Adjust velocity", 10, 70, 15, LIGHTGRAY);
    DrawText("W/S: Increase/decrease mass", 10, 100, 15, LIGHTGRAY);
    DrawText("A/D: Increase/decrease radius", 10, 130, 15, LIGHTGRAY);
    DrawText("Press ESC to exit", 10, 160, 15, LIGHTGRAY);

    // If a body is selected, display its attributes on the top right.
    if (selectedIndex >= 0 && selectedIndex < static_cast<int>(bodies.size())) {
        const Body& b = bodies[selectedIndex];
        char info[128];

        // Introduce delay
        static float elapsedTime = 0.0f;
        elapsedTime += GetFrameTime();
        if (constexpr float delayTime = 0.1f; elapsedTime >= delayTime) {
            dispvelx = b.velocity.x;
            dispvely = b.velocity.y;
            dispposx = b.position.x;
            dispposy = b.position.y;

            elapsedTime = 0.0f;
        }

        //Display velocity
        sprintf(info, "Vel: (%.1f, %.1f)", dispvelx, dispvely);
        DrawText(info, GetScreenWidth() - 280, 40, 20, LIGHTGRAY);

        // Display mass.
        sprintf(info, "Mass: %.1f", b.mass);
        DrawText(info, GetScreenWidth() - 280, 70, 20, LIGHTGRAY);

        // Display radius.
        sprintf(info, "Radius: %.1f", b.radius);
        DrawText(info, GetScreenWidth() - 280, 100, 20, LIGHTGRAY);

        // Display position.
        sprintf(info, "Pos: (%.1f, %.1f)", dispposx, dispposy);
        DrawText(info, GetScreenWidth() - 280, 10, 20, LIGHTGRAY);
    }
    EndDrawing();
}

void OrbitSimulator::run() {
    while (!WindowShouldClose()) {
        const float dt = GetFrameTime() * dtScale;
        handleInput(dt);
        updatePhysics(dt);
        draw();
    }
}