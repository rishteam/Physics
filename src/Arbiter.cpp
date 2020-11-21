#include "Arbiter.h"

Arbiter::Arbiter(Shape* b1_, Shape* b2_)
{
    if (b1_ < b2_)
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
    b1->Collide(this, *b2);

}

//預備好所需要的參數，包含k, delta_v, n, bias
void Arbiter::PreStep(float inv_dt)
{
    // 必須要有個允許穿透值，否則在沒有穿透時計算衝量，會有抖動(jitter)發生
    const float k_allowedPenetration = 0.1f;
    // bangar 修正，是一個介於0~1之間的數字，越小則越慢才修正
    float k_biasFactor = World::positionCorrection ? 0.2f : 0.0f;

    for (int i = 0; i < contactCounter; ++i)
    {
        Contact* c = contacts + i;

        auto body1 = b1;
        auto body2 = b2;

        Vec2 r1 = c->position - body1->position;
        Vec2 r2 = c->position - body2->position;

        //實際計算出兩接觸點法向量的衝量
        float rn1 = Dot(r1, c->normal);
        float rn2 = Dot(r2, c->normal);
        //算出effective mass
        float kNormal = body1->invMass + body2->invMass;
        kNormal += body1->invI * (Dot(r1, r1) - rn1 * rn1) + body2->invI * (Dot(r2, r2) - rn2 * rn2);
        c->massNormal = 1.0f / kNormal;


        //算出切線的法向量衝量
        Vec2 tangent = Cross(c->normal, 1.0f); // ??
        float rt1 = Dot(r1, tangent);
        float rt2 = Dot(r2, tangent);
        float kTangent = body1->invMass + body2->invMass;
        kTangent += body1->invI * (Dot(r1, r1) - rt1 * rt1) + body2->invI * (Dot(r2, r2) - rt2 * rt2);
        c->massTangent = 1.0f /  kTangent;

        // Bias velocity and impulse
        // (bargar / delta_t)
        c->bias = -k_biasFactor * inv_dt * Min(0.0f, c->penetration + k_allowedPenetration);

        // Apply normal + friction impulse
        // 修正衝量大小 + 修正方向(能將兩物體最有效分開的向量)，切線衝量大小 + 切線方向修正
        // Euler-method
        if (World::accumulateImpulses)
        {
            Vec2 P = c->Pn * c->normal + c->Pt * tangent;

            body1->velocity -= body1->invMass * P;
            body1->angularVelocity -= body1->invI * Cross(r1, P);

            body2->velocity += body2->invMass * P;
            body2->angularVelocity += body2->invI * Cross(r2, P);
        }
    }

}

//透過衝量，計算出速度
void Arbiter::ApplyImpulse()
{
    auto body1 = b1;
    auto body2 = b2;
    b1->Collide(this, *b2);

    for (int i = 0; i < contactCounter; ++i)
    {
        // Get contact point
        // 撈出接觸點
        Contact* c = contacts + i;

        // 找出直心與接觸點的向量(r1, r2)
        c->r1 = c->position - body1->position;
        c->r2 = c->position - body2->position;

        // 修正衝量的計算 dot(dv, n)/ kn (p.21)

        // Relative velocity at contact
        Vec2 dv = body2->velocity + Cross(body2->angularVelocity, c->r2) - body1->velocity - Cross(body1->angularVelocity, c->r1);

        // Compute normal impulse
        float vn = Dot(dv, c->normal);

        // massNormal = 1 / kn
        float dPn = c->massNormal * (-vn + c->bias);


        // 修正的衝量都必須介於0以上，否則會越陷越深
        // Max(dPn, 0)
        // accumulateImpulses，會考慮前一幀的衝量，加上要修正的變化量
        if (World::accumulateImpulses)
        {
            // Clamp the accumulated impulse
            float Pn0 = c->Pn; // old
            c->Pn = Max(Pn0 + dPn, 0.0f); // new clamp 0
            dPn = c->Pn - Pn0; // new - old
        }
        else
        {
            dPn = Max(dPn, 0.0f);
        }

        // -----------以下是Apply to Velocity--------------
        // Apply contact impulse to Velocity

        // 以下是正向部分
        Vec2 Pn = dPn * c->normal; // impluse with direction

        body1->velocity -= body1->invMass * Pn;
        body1->angularVelocity -= body1->invI * Cross(c->r1, Pn);

        body2->velocity += body2->invMass * Pn;
        body2->angularVelocity += body2->invI * Cross(c->r2, Pn);

        // Relative velocity at contact
        // 更新兩物體的相對速度(線性、角速度)，丟給切線方向計算
        dv = body2->velocity + Cross(body2->angularVelocity, c->r2) - body1->velocity - Cross(body1->angularVelocity, c->r1);

        // 以下是切線部分
        Vec2 tangent = Cross(c->normal, 1.0f); // why 1.f?
        // dv * n(tangent)
        float vt = Dot(dv, tangent);
        // massTangent = 1 / kn(tangent)
        float dPt = c->massTangent * (-vt);

        if (World::accumulateImpulses)
        {
            // Compute friction impulse
            float maxPt = friction * c->Pn;
            // Clamp friction
            // 摩擦力後的衝量會介於-uPt ~ + uPt之間
            float oldTangentImpulse = c->Pt;
            c->Pt = Clamp(oldTangentImpulse + dPt, -maxPt, maxPt);
            dPt = c->Pt - oldTangentImpulse;
        }
        else
        {
            float maxPt = friction * dPn;
            dPt = Clamp(dPt, -maxPt, maxPt);
        }

        // Apply contact impulse
        Vec2 Pt = dPt * tangent;

        body1->velocity -= body1->invMass * Pt;
        body1->angularVelocity -= body1->invI * Cross(c->r1, Pt);

        body2->velocity += body2->invMass * Pt;
        body2->angularVelocity += body2->invI * Cross(c->r2, Pt);
    }
}

// update the arbiter and calculate the pulse
// 更新arbiter中contact point的數量，以及其中的衝量大小
void Arbiter::update(Contact* newContacts, int numContacts_)
{
//
//    b1->Collide(this, *b2);
//
//    for(int i = 0; i < numContacts_; i++)
////    {
//        Contact* cNew = newContacts + i;
//        int k = -1;
//        if (k > -1)
//            {
//                Contact* c = mergedContacts + i;
//                Contact* cOld = contacts + k;
//                *c = *cNew;
//                if (World::warmStarting)
//                {
//                    c->Pn = cOld->Pn;
//                    c->Pt = cOld->Pt;
//                    c->Pnb = cOld->Pnb;
//                }
//                else
//                {
//                    c->Pn = 0.0f;
//                    c->Pt = 0.0f;
//                    c->Pnb = 0.0f;
//                }
//            }
//            else
//            {
//                mergedContacts[i] = newContacts[i];
//            }
//    }
//    if (World::warmStarting)
//    {
//        c->Pn = cOld->Pn;
//        c->Pt = cOld->Pt;
//        c->Pnb = cOld->Pnb;
//    }
//    else
//    {
//        c->Pn = 0.0f;
//        c->Pt = 0.0f;
//        c->Pnb = 0.0f;
//    }
//    Contact mergedContacts[2];
//
//    for(int i = 0; i < numContacts_; i++)
//    {
//        Contact* cNew = newContacts + i;
//        int k = -1;
//        // 如果前一幀與這一幀的feature value不一樣，才更新下一幀的速率
//        for (int j = 0; j < contactCounter; ++j)
//        {
//            Contact* cOld = contacts + j;
//            if (cNew-> == cOld->penetration)
//            {
//                k = j;
//                break;
//            }
//        }
//
//        if (k > -1)
//        {
//            Contact* c = mergedContacts + i;
//            Contact* cOld = contacts + k;
//            *c = *cNew;
//            if (World::warmStarting)
//            {
//                c->Pn = cOld->Pn;
//                c->Pt = cOld->Pt;
//                c->Pnb = cOld->Pnb;
//            }
//            else
//            {
//                c->Pn = 0.0f;
//                c->Pt = 0.0f;
//                c->Pnb = 0.0f;
//            }
//        }
//        else
//        {
//            mergedContacts[i] = newContacts[i];
//        }
//    }
//
//    for (int i = 0; i < numContacts_; ++i)
//        this->contacts[i] = mergedContacts[i];

}

