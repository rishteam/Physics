#pragma once
#include <vector>
#include <map>
#include "vector_math.h"
#include "Joint.h"

class ArbiterKey;
class Arbiter;
class Shape;

enum class COLLISION {
    SAT,
    GJK
};

class World
{
public:
    World(Vec2 gravity_, float width_, float height_);

    void Add(Shape* body);

    void AddJoints(Joint* joint);

    void Clear();

    void Step(float delta_t);

    void BoardPhase();

    static Vec2 ChangeToPhysicsWorld(const Vec2& ps);

    static Vec2 ConvertWorldToScreen(const Vec2& pw);

    Vec2 gravity;
    static COLLISION collision_type;
    static bool accumulateImpulses;
    static bool warmStarting;
    static bool positionCorrection;
    static float width;
    static float height;
    static Vec2 m_center;
    float timeStep = 1.0 / 60.0f;
    int iterations = 10;

//   TODO: implement QuadTree
    std::vector<Shape*> bodies;
    std::vector<Joint*> joints;
    std::map<ArbiterKey, Arbiter> arbiters;

};
