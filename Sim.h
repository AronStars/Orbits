#ifndef SIM_H
#define SIM_H

#include <vector>
#include "raylib.h"
#include "Body.h"

class OrbitSimulator {
private:
    std::vector<Body> bodies;
    float dispposx;
    float dispposy;
    float dispvelx;
    float dispvely;
    int selectedIndex;
    bool isDragging;
    const float G;         // Gravitational constant
    const float dtScale;   // Time scale factor for simulation

    // Private helper functions.
    static float distance(Vector2 a, Vector2 b);
    [[nodiscard]] int findBodyAtPosition(Vector2 pos) const;
    void handleInput(float dt);
    [[nodiscard]] Vector2 calculateAcceleration(int i, const std::vector<Body> &currentBodies) const;
    void handleCollisions();
    void updatePhysics(float dt);
    void draw();

    // Computes predicted future paths of bodies.
    [[nodiscard]] std::vector<std::vector<Vector2>> computePredictedPaths() const;

public:
    // Getter: returns a constant reference to the vector of bodies.
    [[nodiscard]] const std::vector<Body>& getBodies() const;
    
    // Non-const getter: returns a reference to the vector of bodies.
    std::vector<Body>& getBodies();
    
    // Setter: sets the bodies vector.
    void setBodies(const std::vector<Body> &newBodies);

    // Add a new body to the simulator.
    void addBody(const Vector2 &position, const Vector2 &velocity, float mass, float radius);

    OrbitSimulator();
    ~OrbitSimulator();
    void run();
};

#endif // SIM_H