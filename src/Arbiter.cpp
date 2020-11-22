#include "Arbiter.h"

Arbiter::Arbiter(Shape* b1_, Shape* b2_)
{
    if (b1_ > b2_)
    {
        b1 = b1_;
        b2 = b2_;
    }
    else
    {
        b1 = b2_;
        b2 = b1_;
    }

    // Calculate Contact point
    b1->SetMatrix(b1->angle);
    b2->SetMatrix(b2->angle);
    // Find Contact Points
    b1->Collide(*this, *b2);
}

//預備好所需要的參數，包含k, delta_v, n, bias
void Arbiter::PreStep(float inv_dt, Vec2 gravity)
{
    auto &body1 = b2;
    auto &body2 = b1;
//    // 必須要有個允許穿透值，否則在沒有穿透時計算衝量，會有抖動(jitter)發生
//    const float k_allowedPenetration = 0.5f;
//    // bangar 修正，是一個介於0~1之間的數字，越小則越慢才修正
//    float k_biasFactor = World::positionCorrection ? 0.2f : 0.0f;
//
//    for (int i = 0; i < contactCounter; ++i)
//    {
//        // 兩物體質心到接觸點的向量
//        Vec2 r1 = contacts[i].position - body1->position;
//        Vec2 r2 = contacts[i].position - body2->position;
//
//        // 實際計算出兩接觸點法向量的衝量
//        float rn1 = Dot(r1, contacts[i].normal);
//        float rn2 = Dot(r2, contacts[i].normal);
//        // 算出effective mass
//        float kNormal = body1->invMass + body2->invMass;
//        kNormal += body1->invI * (Dot(r1, r1) - rn1 * rn1) + body2->invI * (Dot(r2, r2) - rn2 * rn2);
//        contacts[i].massNormal = 1.0f / kNormal;
//
//
//
//        // -----------------以下是切線相關-----------------
//        // 算出切線的衝量，那個Cross是假的Cross
//        Vec2 tangent = Cross(contacts[i].normal, 1.0f);
//        float rt1 = Dot(r1, tangent);
//        float rt2 = Dot(r2, tangent);
//        float kTangent = body1->invMass + body2->invMass;
//        kTangent += body1->invI * (Dot(r1, r1) - rt1 * rt1) + body2->invI * (Dot(r2, r2) - rt2 * rt2);
//        contacts[i].massTangent = 1.0f /  kTangent;
//
//        // Bias velocity and impulse
//        // (bargar / delta_t)
//        contacts[i].bias = -k_biasFactor * inv_dt * Min(0.0f, contacts[i].penetration + k_allowedPenetration);
//
//        // Apply normal + friction impulse
//        // 修正衝量大小 + 修正方向(能將兩物體最有效分開的向量)，切線衝量大小 + 切線方向修正
//        // Euler-method
//        if (World::accumulateImpulses)
//        {
//            Vec2 P = contacts[i].Pn *  contacts[i].normal + contacts[i].Pt * tangent;
//
//            body1->velocity -= body1->invMass * P;
//            body1->angularVelocity -= body1->invI * Cross(r1, P);
//
//            body2->velocity += body2->invMass * P;
//            body2->angularVelocity += body2->invI * Cross(r2, P);
//        }
//    }

    // Calculate average restitution
    e = std::min( b1->restitution, b2->restitution );

    // Calculate static and dynamic friction
    sf = std::sqrt( b1->staticFriction * b2->staticFriction );
    df = std::sqrt( b1->dynamicFriction * b2->dynamicFriction );

    for(int i = 0; i < contactCounter; ++i)
    {
        // Calculate radii from COM to contact
        Vec2 ra = contacts[i].position - b1->position;
        Vec2 rb = contacts[i].position - b2->position;

        Vec2 rv = b2->velocity + Cross( b2->angularVelocity, rb ) -
                  b1->velocity - Cross( b1->angularVelocity, ra );


        // Determine if we should perform a resting collision or not
        // The idea is if the only thing moving this object is gravity,
        // then the collision should be performed without any restitution
        if(rv.LenSqr( ) < (inv_dt * gravity).LenSqr( ) + EPSILON)
            e = 0.0f;
    }

}

//透過衝量，計算出速度
void Arbiter::ApplyImpulse()
{
    auto &body1 = b2;
    auto &body2 = b1;

//    for (int i = 0; i < contactCounter; ++i)
//    {
//        // 找出直心與接觸點的向量(r1, r2)
//        contacts[i].r1 = contacts[i].position - body1->position;
//        contacts[i].r2 = contacts[i].position - body2->position;
//
//        // 修正衝量的計算 dot(dv, n)/ kn (p.21)
//        // Relative velocity at contact
//        // Delta V
//        Vec2 dv = body2->velocity + Cross(body2->angularVelocity, contacts[i].r2)
//                - body1->velocity - Cross(body1->angularVelocity, contacts[i].r1);
//
//        // Compute normal impulse
//        // DeltaV * n / kn = Pn
//        float vn = Dot(dv, contacts[i].normal);
//
//        // massNormal = 1 / kn
//        float dPn = contacts[i].massNormal * (-vn + contacts[i].bias);
//
//        // 修正的衝量都必須介於0以上，否則會越陷越深
//        // Max(dPn, 0)
//        // accumulateImpulses，會考慮前一幀的衝量，加上要修正的變化量
//        if (World::accumulateImpulses)
//        {
//            // Clamp the accumulated impulse
//            float Pn0 =  contacts[i].Pn; // old
//            contacts[i].Pn = Max(Pn0 + dPn, 0.0f); // new clamp 0
//            dPn =  contacts[i].Pn - Pn0; // new - old
//        }
//        else
//        {
//            dPn = Max(dPn, 0.0f);
//        }
//
//        // -----------以下是Apply to Velocity--------------
//        // Apply contact impulse to Velocity
//
//        // 以下是正向部分
//        Vec2 Pn = dPn * contacts[i].normal; // Impulse with direction
//
//        body1->velocity -= body1->invMass * Pn;
//        body1->angularVelocity -= body1->invI * Cross(contacts[i].r1, Pn);
//
//        body2->velocity += body2->invMass * Pn;
//        body2->angularVelocity += body2->invI * Cross(contacts[i].r2, Pn);
//
//        // 以下是切線部分
//        // Relative velocity at contact
//        // 更新兩物體的相對速度(線性、角速度)，丟給切線方向計算
//        dv = body2->velocity + Cross(body2->angularVelocity, contacts[i].r2) - body1->velocity - Cross(body1->angularVelocity,  contacts[i].r1);
//
//        Vec2 tangent = Cross(contacts[i].normal, 1.0f); // why 1.f?
//        // dv * n(tangent)
//        float vt = Dot(dv, tangent);
//        // massTangent = 1 / kn(tangent)
//        float dPt = contacts[i].massTangent * (-vt);
//
//        if (World::accumulateImpulses)
//        {
//            // Compute friction impulse
//            float maxPt = friction * contacts[i].Pn;
//            // Clamp friction
//            // 摩擦力後的衝量會介於-uPt ~ + uPt之間
//            float oldTangentImpulse = contacts[i].Pt;
//            contacts[i].Pt = Clamp(oldTangentImpulse + dPt, -maxPt, maxPt);
//            dPt = contacts[i].Pt - oldTangentImpulse;
//        }
//        else
//        {
//            float maxPt = friction * dPn;
//            dPt = Clamp(dPt, -maxPt, maxPt);
//        }
//
//        // Apply contact impulse
//        Vec2 Pt = dPt * tangent;
//
//        body1->velocity -= body1->invMass * Pt;
//        body1->angularVelocity -= body1->invI * Cross(contacts[i].r1, Pt);
//
//        body2->velocity += body2->invMass * Pt;
//        body2->angularVelocity += body2->invI * Cross(contacts[i].r2, Pt);
//    }

    for(int i = 0; i < contactCounter; ++i)
    {
        // Calculate radii from COM to contact
        Vec2 ra = contacts[i].position - body1->position;
        Vec2 rb = contacts[i].position - body2->position;

        // Relative velocity
        Vec2 rv = body2->velocity + Cross( body2->angularVelocity, rb ) -
                  body1->velocity - Cross( body1->angularVelocity, ra );

        // Relative velocity along the normal
        float contactVel = Dot( rv, normal );

        // Do not resolve if velocities are separating
        if(contactVel > 0)
            return;

        float raCrossN = Cross( ra, normal );
        float rbCrossN = Cross( rb, normal );
        float invMassSum = body1->invMass + body2->invMass + raCrossN * raCrossN * body1->invI + rbCrossN * rbCrossN * body2->invI;

        // Calculate impulse scalar
        float j = -(1.0f + e) * contactVel;
        j /= invMassSum;
        j /= (float)contactCounter;

        // Apply impulse
        Vec2 impulse = normal * j;

        body1->velocity += body1->invMass * -impulse;
        body1->angularVelocity += body1->invI * Cross( ra, -impulse );

        body2->velocity += body2->invMass * impulse;
        body2->angularVelocity += body2->invI * Cross( rb, impulse );


        // Friction impulse
        rv = body2->velocity + Cross( body2->angularVelocity, rb ) -
             body1->velocity - Cross( body1->angularVelocity, ra );

        Vec2 t = rv - (normal * Dot( rv, normal ));
        t.Normalize( );

        // j tangent magnitude
        float jt = -Dot( rv, t );
        jt /= invMassSum;
        jt /= (float)contactCounter;

        // Don't apply tiny friction impulses
        if( jt < EPSILON)
            return;

        // Coulumb's law
        Vec2 tangentImpulse;
        if(std::abs( jt ) < j * sf)
            tangentImpulse = t * jt;
        else
            tangentImpulse = t * -j * df;

        // Apply friction impulse
        body1->velocity += body1->invMass * -tangentImpulse;
        body1->angularVelocity += body1->invI * Cross( ra, -tangentImpulse );

        body2->velocity += body2->invMass * tangentImpulse;
        body2->angularVelocity += body2->invI * Cross( rb, tangentImpulse );
    }
}

// update the arbiter and calculate the pulse
// 更新arbiter中contact point的數量，以及其中的衝量大小
void Arbiter::Update()
{
    if(!b1->Collide(*this, *b2))
    {
        contactCounter = 0;
    }
}

