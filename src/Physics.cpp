#include "Physics.h"

Physics::Physics(){
    mass = 10;
    velocity = Vec2(0.0f, 0.0f);
    angularVelocity = 0;
    force = Vec2(0.0, 0.0);
    torque = 0;
    friction = 0.2f;
    I = 0;
    restitution = 0;
}

void Physics::AddForce(const Vec2& f)
{
    force += f;
}

void Physics::ComputeForce(float delta_t, Vec2 gravity)
{
    if (this->invMass == 0.0f)
        return;
    else
    {
        this->velocity += delta_t * (gravity + this->invMass * this->force);
        this->angularVelocity += delta_t * this->invI * this->torque;
    }
}

void Physics::IntegrateVelocities(float delta_t)
{
    auto tmp = delta_t * velocity;
    this->position += tmp;
    auto tmp2 = delta_t * angularVelocity;
    this->angle += tmp2;

    this->force.Set(0.0f, 0.0f);
    this->torque = 0.0f;
}

