#include "World.h"


World::World(Vec2 gravity_, float width_, float height_)
{
    gravity = gravity_;
    width = width_;
    height = height_;
}

void World::Clear()
{
    bodies.clear();
}

void World::Add(Shape* body)
{
    bodies.push_back(body);
}

void World::Step(float delta_t)
{
    // Compute forces.
    for(int i = 0; i < bodies.size(); i++)
    {
        Box* box = dynamic_cast<Box*>(bodies.at(i));
        auto tmp = Vec2(box->getPosition().x, box->getPosition().y);
        box->TransformPhysicsCoordinate(ChangeToPhysicsWorld(tmp), box->getRotation());
        box->ComputeForce(delta_t, gravity);
    }
    // Integrate Velocities
    for(int i = 0; i < bodies.size(); i++)
    {
        Box* box = dynamic_cast<Box*>(bodies.at(i));
        box->IntegrateVelocities(delta_t);
        auto tmp2 = box->getPhysicsData();
        box->setPosition(ConvertWorldToScreen(tmp2.first));
        box->setRotation(tmp2.second);
    }
}

void World::BoardPhase()
{
    for(int i = 0; i < bodies.size(); i++)
    {
        for(int j = i+1; j < bodies.size(); j++)
        {
            Box* box1 = dynamic_cast<Box*>(bodies.at(i));
            Box* box2 = dynamic_cast<Box*>(bodies.at(j));
            if(box1->getMass() == 0.0f && box2->getMass() == 0.0f)
                continue;
            //add in Arbiter

            Arbiter newArb(bodies[i], bodies[j]);
            ArbiterKey key(bodies[i], bodies[j]);
            auto iter = arbiters.find(key);

            if(box1->isCollide(*box2))
            {
                if (iter == arbiters.end())
                {
                    //TODO: add pair
                    arbiters.insert()
                }
                else
                {
                    iter->second.update(newArb.contacts, newArb.numContacts);
                }
            }
            else
            {
                arbiters.erase(key);
            }
        }

    }

}

Vec2 World::ChangeToPhysicsWorld(const Vec2& ps)
{
    float w = float(width/2);
    float h = float(height/2);
    float u = ps.x / w;
    float v = (h - ps.y) / h;

    float ratio = w / h;
    Vec2 extents(ratio * 25.0f, 25.0f);

    Vec2 lower = m_center - extents;
    Vec2 upper = m_center + extents;

    Vec2 pw(0.0, 0.0);
    pw.x = (1.0f - u) * lower.x + u * upper.x;
    pw.y = (1.0f - v) * lower.y + v * upper.y;
    return pw;
};

Vec2 World::ConvertWorldToScreen(const Vec2& pw)
{
    float w = float(width/2);
    float h = float(height/2);
    float ratio = w / h;
    Vec2 extents(ratio * 25.0f, 25.0f);

    Vec2 lower = m_center - extents;
    Vec2 upper = m_center + extents;

    float u = (pw.x - lower.x) / (upper.x - lower.x);
    float v = (pw.y - lower.y) / (upper.y - lower.y);

    Vec2 ps(0.0, 0.0);
    ps.x = u * w;
    ps.y = (1.0f - v) * h;
    return ps;
};

