#include "World.h"

bool World::accumulateImpulses = true;
bool World::warmStarting = true;
bool World::positionCorrection = true;
float World::width;
float World::height;
Vec2 World::m_center = Vec2(0, 20);

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
    float inv_dt = delta_t > 0.0f ? 1.0f / delta_t : 0.0f;

    //change the physics coordinate
    for(int i = 0; i < bodies.size(); i++)
    {
        Box* box = dynamic_cast<Box*>(bodies.at(i));
        box->TransformPhysicsCoordinate(box->getPosition().x, box->getPosition().y, box->getwidth(), box->getheight(),box->getRotation());
    }

    BoardPhase();

    // Compute forces.
    for(int i = 0; i < bodies.size(); i++)
    {
        Box* box = dynamic_cast<Box*>(bodies.at(i));
        box->ComputeForce(delta_t, gravity);
    }

    //pre-step arbiter
    for (auto arb = arbiters.begin(); arb != arbiters.end(); ++arb)
    {
        arb->second.PreStep(inv_dt);
    }

    //apply impulse
    for (auto arb = arbiters.begin(); arb != arbiters.end(); ++arb) {
        arb->second.ApplyImpulse();
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

//            fmt::print("box1:{} {}\n",box1->position.x, box1->position.y);
//            fmt::print("box2:{} {}\n",box2->position.x, box2->position.y);

            if(box1->getMass() == 0.0f && box2->getMass() == 0.0f)
                continue;

            //add in Arbiter
            Arbiter newArb(bodies[i], bodies[j]);
            ArbiterKey key(bodies[i], bodies[j]);
            auto iter = arbiters.find(key);


            // collision
            if(box1->isCollide(*box2))
            {
                //add new arbiter
                if (iter == arbiters.end())
                {
                    arbiters.insert(std::pair<ArbiterKey, Arbiter>(key, newArb));
                }
                //update arbiter
                else
                {
                    iter->second.update(newArb.contacts, newArb.numContacts);
                }
            }
            //兩物體沒接觸點，代表沒碰撞
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

