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
    for(auto ptr : bodies)
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
    bodies.clear();
    joints.clear();
    arbiters.clear();
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

    //Boardphase detection
    BoardPhase();

    // Compute forces.
    for(int i = 0; i < bodies.size(); i++)
    {
        if (bodies[i]->invMass == 0.0f)
            continue;
        bodies[i]->ComputeForce(delta_t, gravity);
    }

    //Pre-step arbiter
//    for (auto jit : joints)
//    {
//        jit->PreStep(inv_dt);
//    }

    for (auto arb = arbiters.begin(); arb != arbiters.end(); ++arb)
    {
        arb->second.PreStep(inv_dt);
    }

    for (int i = 0; i < this->iterations; ++i)
    {
        // Apply impulse
        for (auto arb = arbiters.begin(); arb != arbiters.end(); ++arb) {
            arb->second.ApplyImpulse();
        }
        // Joint
//        for (auto jit : joints)
//        {
//            jit->ApplyImpulse();
//        }
    }

    // Integrate Velocities
    for(int i = 0; i < bodies.size(); i++)
    {
        bodies.at(i)->IntegrateVelocities(delta_t);
    }
}

void World::BoardPhase()
{
    for(int i = 0; i < bodies.size(); i++)
    {
        Shape* b1 = bodies[i];
        for(int j = i+1; j < bodies.size(); j++)
        {
            Shape* b2 = bodies[j];
            if(b1->invMass == 0.0f && b2->invMass == 0.0f)
                continue;

            //add in Arbiter
            Arbiter newArb(b1, b2);
            ArbiterKey key(b1, b2);

            if (newArb.contactCounter > 0)
            {
                auto iter = arbiters.find(key);
                if (iter == arbiters.end())
                {
                    arbiters.insert(std::make_pair(key, newArb));
                }
                else
                {
                    iter->second.Update();
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

