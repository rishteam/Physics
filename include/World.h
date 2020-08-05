#pragma once
#include <vector>
#include <map>
#include "vector_math.h"
#include "Shape.h"
#include "Box.h"
#include "Circle.h"
#include "Polygon.h"
#include "Arbiter.h"

class ArbiterKey;
class Arbiter;

class World
{
public:
    World(Vec2 gravity_, float width_, float height_);

    void Add(Shape* body);

    void Clear();

    void Step(float delta_t);

    void BoardPhase();

    static Vec2 ChangeToPhysicsWorld(const Vec2& ps);

    Vec2 ConvertWorldToScreen(const Vec2& pw);

    Vec2 gravity;
    static bool accumulateImpulses;
    static bool warmStarting;
    static bool positionCorrection;
    static float width;
    static float height;
    static Vec2 m_center;
    float timeStep = 1.0 / 60.0f;
    int iterations = 10;
    std::vector<Shape*> bodies;
    std::map<ArbiterKey, Arbiter> arbiters;

};
