#include <vector>
#include <map>
#include "vector_math.h"
#include "Shape.h"
#include "Box.h"
#include "Circle.h"
#include "Polygon.h"

#ifndef SAT_COLLISION_WORLD_H
#define SAT_COLLISION_WORLD_H

class World
{
public:
    World(Vec2 gravity_, float width_, float height_);

    void Add(Shape* body);

    void Clear();

    void Step(float delta_t);

    void BoardPhase();

    Vec2 ChangeToPhysicsWorld(const Vec2& ps);

    Vec2 ConvertWorldToScreen(const Vec2& pw);

    Vec2 gravity;
    static bool accumulateImpulses;
    static bool warmStarting;
    static bool positionCorrection;
    float timeStep = 1.0 / 60.0f;
    float width = 0;
    float height = 0;
    Vec2 m_center = Vec2(width/2, height/2);
    std::vector<Shape*> bodies;

};



#endif //SAT_COLLISION_WORLD_H
