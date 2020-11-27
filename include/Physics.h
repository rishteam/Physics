#include "vector_math.h"
#include "World.h"

class Physics
{
public:
    Physics();
    ~Physics() = default;
    void IntegrateVelocities(float delta_t);
    void ComputeForce(float delta_t, Vec2 gravity);
    void AddForce(const Vec2& f);

    //物理世界座標
    Vec2 position;
    //寬高
    Vec2 wh;
    //角速度
    float angularVelocity;
    //角度
    float angle;
    //速度
    Vec2 velocity;
    //力
    Vec2 force;
    //力矩
    float torque;
    //質量，質量倒數
    float mass, invMass;
    //慣性矩，慣性矩倒數
    float I, invI;


    float restitution = 0.2f;
    float staticFriction = 0.5f;
    float dynamicFriction = 0.3f;
};
