#include "World.h"
#include "Arbiter.h"
#include "Circle.h"
#include "Polygon.h"

bool World::accumulateImpulses = false;
bool World::warmStarting = false;
bool World::positionCorrection = true;
float World::width = 1280;
float World::height = 720;
COLLISION World::collision_type = COLLISION::SAT;
Vec2 World::m_center = Vec2(0, 0);

World::World(Vec2 gravity_, float width_, float height_)
{
    gravity = gravity_;
    width = width_;
    height = height_;
}

void World::Clear()
{
    //release space
    for(auto &ptr : bodies)
    {
        switch(ptr->type)
        {
            case Shape::Type::Box:
            {
                Box *box = dynamic_cast<Box *>(ptr);
                delete box;
                break;
            }
            case Shape::Type::Circle:
            {
                Circle *circle = dynamic_cast<Circle *>(ptr);
                delete circle;
                break;
            }
            case Shape::Type::Polygon:
            {
                Polygon * poly = dynamic_cast<Polygon *>(ptr);
                delete poly;
                break;
            }
        }
    }
    arbiters.clear();
    bodies.clear();
    joints.clear();
}

void World::Add(Shape* body)
{
    bodies.push_back(body);
}

void World::AddJoints(Joint* joint)
{
    joints.push_back(joint);
}

void World::Step(float delta_t)
{
    float inv_dt = delta_t > 0.0f ? 1.0f / delta_t : 0.0f;

    // BoardPhase detection
    BoardPhase();

    // Compute forces.
    for(int i = 0; i < bodies.size(); i++)
    {
        if (bodies.at(i)->invMass == 0.0f)
            continue;
        bodies.at(i)->ComputeForce(delta_t, gravity);
    }

    for (auto arb = arbList.begin(); arb != arbList.end(); ++arb)
    {
        arb->PreStep(inv_dt, gravity);
    }

    for (int i = 0; i < this->iterations; ++i)
    {
        // Apply impulse
        for(int j = 0; j < arbList.size( ); ++j)
        {
            arbList[j].ApplyImpulse();
        }
//        for (auto arb = arbList.begin(); arb != arbList.end(); ++arb)
//        {
//            arb->ApplyImpulse();
//        }
    }

    // Integrate Velocities
    for(int i = 0; i < bodies.size(); i++)
    {
        bodies.at(i)->IntegrateVelocities(delta_t);
    }

//    // Correct positions
    for(int i = 0; i < arbList.size( ); ++i)
    {
        arbList[i].PositionalCorrection();
    }

}

void World::BoardPhase()
{
    arbList.clear();
    for(int i = 0; i < bodies.size(); i++)
    {
        for(int j = i+1; j < bodies.size(); j++)
        {
            if(bodies[i]->invMass == 0.0f && bodies[j]->invMass == 0.0f)
                continue;

            //add in Arbiter
            Arbiter newArb(bodies[i], bodies[j]);
            newArb.Solve();

            if (newArb.contactCounter)
            {
                arbList.emplace_back(newArb);
            }

//            ArbiterKey key(bodies[i], bodies[j]);
//            if (newArb.contactCounter > 0)
//            {
//                auto iter = arbiters.find(key);
//                if (iter == arbiters.end())
//                {
//                    arbiters.insert(std::make_pair(key, newArb));
//                }
//                else
//                {
//                    iter->second.Update();
//                }
//            }
//            else
//            {
//                arbiters.erase(key);
//            }
        }
    }
}

Vec2 World::ChangeToPhysicsWorld(const Vec2& ps)
{
    float w = float(width);
    float h = float(height);
    float u = ps.x / w;
    float v = (h - ps.y) / h;

    float ratio = w / h;
    Vec2 extents(ratio * 25.0f, 25.0f);

    Vec2 lower = m_center - extents;
    Vec2 upper = m_center + extents;


    Vec2 pw;
    pw.x = (1.0f - u) * lower.x + u * upper.x;
    pw.y = (1.0f - v) * lower.y + v * upper.y;
    return pw;
};

Vec2 World::ConvertWorldToScreen(const Vec2& pw)
{
    float w = float(width);
    float h = float(height);
    float ratio = w / h;

    Vec2 extents(ratio * 25.0f, 25.0f);

    Vec2 lower = m_center - extents;
    Vec2 upper = m_center + extents;

    float u = (pw.x - lower.x) / (upper.x - lower.x);
    float v = (pw.y - lower.y) / (upper.y - lower.y);

    Vec2 ps;
    ps.x = u * w;
    ps.y = (1.0f - v) * h;
    return ps;
};

