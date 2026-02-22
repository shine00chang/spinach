#include "constraint.h"
#include "body.h"
#include "view.h"
#include <map>

// Impulse table for sequential impulse resolution state
std::map<Body*, Impulse> impulses;
Impulse getJ (std::shared_ptr<Body> b) {
    return impulses.count(b.get()) ? impulses[b.get()] : Impulse{ Vec2(), 0.0 };
}

void accumulateJ (std::shared_ptr<Body> b, Vec2 v, Vec2 impulse) {
    auto [J, L] = impulses[b.get()];
    Vec3 r = v-b->getPos();
    Vec3 dL = r ^ impulse;
    impulses[b.get()] = Impulse{J + impulse, L + dL.z};
};


// sequential impulse
// do not accmulate impulse into velocity
// accmulate impulse for each body
// then apply impulse onto velocity
// convergence checker needs to apply the accumulated velocity and check from there.
// we do not apply it in collision
void sequentialImpulse (Constraints constraints, const double dt) 
{ 
    impulses.clear();
        
    int iteration = 0;

    while (!constraints.empty()) 
    {
        std::list<std::shared_ptr<Constraint>> nconstraints;

        // Resolve collisions: accumulate impulse
        for (auto& constraint: constraints) {
            constraint->resolve(dt);
        }

        // Check Convergence with current accumulated impulse
        // Remove if converged
        for (auto& constraint: constraints) {
            if (!constraint->converged(iteration, getJ(constraint->b2),getJ(constraint->b2))) {
                nconstraints.push_back(constraint);
            }
        }

        constraints = std::move(nconstraints);
        iteration++;
    }

    // Apply impulse now
    for (auto [body, impulse] : impulses) {
        auto [J, L] = impulse;
        std::cout << "J:\t" << J << "\tL:\t" << L << std::endl;
        body->impulse(J, L);
    }

    if (iteration)
        std::cout << "iterations: " << iteration << std::endl;
}

// Returns impulse. Positive for penetrating body (b2)
Vec2 ContactConstraint::resolve (const double dt) 
{
    // http://www.mft-spirit.nl/files/articles/ImpulseSolverBrief.pdf
    // https://raphaelpriatama.medium.com/sequential-impulses-explained-from-the-perspective-of-a-game-physics-beginner-72a37f6fea05
    // (Pa - Pb) * n = 0
    // n * d/dt (Pa - Pb) + (Pa - Pb) * dn/dt = 0
    // n * (Vrel) = 0
    // n * (va + wa x ra - vb - wb x rb) = 0  
    // (n  n x ra  -n  -n x rb) * (V + dV) = 0 V includes both A and B
    // dV = J * lamb, that is, dV is in the direction of N
    // J = (n  n x ra  -n  -n x rb)
    // J * V + J * dV = 0
    // J * V + J * dV = 0
    // lamb = J V / J J^t dt
    // P = J^t lamb
    // TODO: P is then clamped such that sum(P) over all iterations > 0
    // NOTE: right now, we are assuming 1 iteration

    Body& b1 = *this->b1;
    Body& b2 = *this->b2;
    Vec3 n = Vec3(this->norm);

    // Calculate Radius
    Vec3 r1 = Vec3(this->v1 - b1.getPos());
    Vec3 r2 = Vec3(this->v2 - b2.getPos());

    // Construct Jacobian
    const Vec12 J = Vec12(-n, n^r1, n, -n^r2);
    const Vec12 V = Vec12(Vec3(b1.getVelo()), Vec3(0, 0, b1.getAngVelo()), Vec3(b2.getVelo()), Vec3(0, 0, b2.getAngVelo()));
    const Vec12 MinvJ_tp = Vec12(
            -n * b1.getInvMass(),
            n^r1 * b1.getInvInertia(),
            n * b2.getInvMass(),
            -n^r2 * b2.getInvInertia());

    // Calculate Bias
    double b = 0;
    {   // Baumgarte Stabilization
        // adds energy to produce impulse to separate penetration.
        // energy is proportional to the positional penetration. 
        // allows set bit of penetration (slop) so as to not introduce energy when the system is near stable.
        // NOTE: not sure why its negative.. isn't it adding energy? might be the sign depending on which body its applied to
        double Baumgarte = 0.4;
        double d = (v1-v2) * norm;
        double slopallowance = 2;
        d = std::max(d-slopallowance, 0.0);
        b += - Baumgarte * d / dt;
    }
    {   // Restitution
        // adds energy proportional to the relevative normal velocity, to produce 'bounce'
        double restitution = 0.1;
        double vreln = J * V;
        double slopallowance = 5;
        vreln = std::max(vreln-slopallowance, 0.0);
        b += restitution * vreln;
    }

    // Calculate resolutionm impulse 
    Vec2 impulseN;
    {
        double num = J * V + b;
        double den = dt * (J * MinvJ_tp);
        double lambda = - num / den;
        lambda = std::max(lambda, 0.0);
        impulseN = this->norm * dt * lambda;
    }

    // Calculate Friction
    // Friction Impulse is a fraction of the relative tangent velocity 
    // clamped by the normal impulse times coeffecient of friction
    Vec2 impulseT;
    {
        const Vec3 t = Vec3(-this->norm.y, this->norm.x, 0);
        const Vec12 Jt = Vec12(-t, t^r1, t, -t^r2);
        const Vec12 MinvJt_tp = Vec12(
            -t * b1.getInvMass(),
            t^r1 * b1.getInvInertia(),
            t * b2.getInvMass(),
            -t^r2 * b2.getInvInertia());

        double num = Jt * V;
        double den = dt * (Jt * MinvJt_tp);
        double lambda = - num / den;

        double mu = 0.5;
        double mag = std::clamp(dt * lambda, -mu * impulseN.mag(), mu * impulseN.mag());
        impulseT = Vec2(t.x * mag, t.y * mag);
    }

    const Vec2 impulse = impulseN + impulseT;
    std::cout << "impulse:\t" << impulse << std::endl;
    
    accumulateJ(this->b1, v1, -impulse);
    accumulateJ(this->b2, v2,  impulse);

    return impulse;
}

bool ContactConstraint::converged(const int iteration, Impulse j1, Impulse j2)
{
    if (iteration >= 4) 
        return true;

    // C: contact points touching

    // Calculate contact points after impulse
    // nv = v + dp
    // rotate nv by angvelo * dt 


    auto pointVelocity = [](Vec2 p, Vec2 v, double angV, Vec2 bp) {
        Vec3 r = p-bp;
        Vec3 av = Vec3(0,0,angV) ^ r;
        return v + Vec2(av.x, av.y);
    };

    auto applyImpulse = [](const Body& body, Vec2 J, double L) {
        Vec2 nv = body.getVelo() + (J * body.getInvMass());
        double nw = body.getAngVelo() + body.getInvInertia() * L;
        return std::make_pair(nv,nw);
    };


    auto [nv1, nw1] = applyImpulse(*b1, j1.J, j1.L);
    auto [nv2, nw2] = applyImpulse(*b2, j2.J, j2.L);

    std::cout << "velocity, angvelocity:\t" << b1->getVelo() << ",\t" << b1->getAngVelo() << std::endl;
    std::cout << "velocity, angvelocity:\t" << b2->getVelo() << ",\t" << b2->getAngVelo() << std::endl;
    std::cout << "velocity, angvelocity, v1:\t" << nv1 << ",\t" << nw1 << ",\t" << (v1-b1->getPos()) << std::endl;
    std::cout << "velocity, angvelocity, v2:\t" << nv2 << ",\t" << nw2 << ",\t" << (v2-b2->getPos()) << std::endl;

    Vec2 delP = v1-v2;
    Vec2 delV = pointVelocity(v1, nv1, nw1, b1->getPos()) + pointVelocity(v2, nv2, nw2, b2->getPos());

    injectDebugEffect(std::make_shared<VectorEffect>(v1, pointVelocity(v1, nv1, nw1, b1->getPos()) * 0.1, Red));
    injectDebugEffect(std::make_shared<VectorEffect>(v2, pointVelocity(v2, nv2, nw2, b2->getPos()) * 0.1, Red));

    std::cout << "delP mag: " << delP.mag() << "\tdelV * n: " << (delV * norm) << std::endl;

    //return del.mag() < 1;
    return true;
}
