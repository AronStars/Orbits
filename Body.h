#ifndef BODY_H
#define BODY_H

#include "raylib.h"

class Body {
public:
    Vector2 position;
    Vector2 velocity;
    float mass;
    float radius; // For drawing and hit detection

    Body(Vector2 position, Vector2 velocity, float mass, float radius);
};

#endif // BODY_H