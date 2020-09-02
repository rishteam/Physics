
#include "Physics.h"

Physics::Physics(){
    mass = 0;
    velocity = Vec2(1.0, -1.0);
    angularVelocity = 0;
    force = Vec2(0.0, 0.0);
    torque = 0;
}

std::pair<Vec2, float> Physics::getPhysicsData()
{
    return std::make_pair(position, angle);
}

float Physics::getMass()
{
    return mass;
}

Vec2 Physics::getwh()
{
    return wh;
}

float Physics::getfriction()
{
    return friction;
}


void Physics::AddForce(const Vec2& f)
{
    force += f;
}

void Physics::TransformPhysicsCoordinate(float x, float y, float w, float h, float angle_)
{
    Vec2 ul = World::ChangeToPhysicsWorld(Vec2(x - w/2, y - h/2));
    Vec2 ur = World::ChangeToPhysicsWorld(Vec2(x + w/2, y - h/2));
    Vec2 dl = World::ChangeToPhysicsWorld(Vec2(x - w/2, y + h/2));
    Vec2 dr = World::ChangeToPhysicsWorld(Vec2(x + w/2, y + h/2));

    position = World::ChangeToPhysicsWorld(Vec2(x, y));
    wh = Vec2(ur.x - ul.x, ur.y - dr.y);
    I = mass * (wh.x * wh.x + wh.y * wh.y) / 12.0f;
    invI = 1.0f / I;
    this->angle = -degreesToRadians(angle_);
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
    position += tmp;
    auto tmp2 = delta_t * angularVelocity;
    angle += tmp2;

    this->force.Set(0.0f, 0.0f);
    this->torque = 0.0f;
}

