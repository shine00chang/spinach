#include "constraint.h"
#include "body.h"
#include "view.h"
#include <map>
#include <memory>

struct Subject {
    Vec3 p;
    double o;
    Vec3 v;
    Vec3 w;
    std::shared_ptr<Body> b;
};

Subject subjectFrom (std::shared_ptr<Body> body) {
    return Subject {
        .p = body->getPos(),
        .o = body->getOrientation(),
        .v = body->getVelo(),
        .w = Vec3(0, 0, body->getAngVelo()),
        .b = body
    };
}

void subjectTo (Subject& subject, Impulse J, double dt) {
    subject.v = subject.v + J.J * subject.b->getInvMass();
    subject.w = subject.w + Vec3(0,0,subject.b->getInvInertia() * J.L);
}

// Impulse table for sequential impulse resolution state
std::map<Body*, Impulse> impulses;
Impulse getJ (std::shared_ptr<Body> b) {
    return impulses.count(b.get()) ? impulses[b.get()] : Impulse{ Vec2(), 0.0 };
}

// accumulates impulse J at point V in terms for J and L about the center of B.
// V is in absolute coordinates
void accumulateJ (std::shared_ptr<Body> b, Vec3 v, Vec2 impulse) {
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
            if (!constraint->converged(iteration, getJ(constraint->b2),getJ(constraint->b2), dt)) {
                nconstraints.push_back(constraint);
            }
        }

        constraints = std::move(nconstraints);
        iteration++;
    }

    // Apply impulse now
    for (auto [body, impulse] : impulses) {
        auto [J, L] = impulse;
        body->impulse(J, L);
    }

    if (iteration)
        std::cerr << "iterations: " << iteration << std::endl;
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
        double Baumgarte = 0.4;
        double d = (v1-v2) * norm;
        double slopallowance = 2;
        d = std::max(d-slopallowance, 0.0);
        b += - Baumgarte * d / dt;
    }
    {   // Restitution
        // adds energy proportional to the relevative normal velocity, to produce 'bounce'
        double restitution = 0.0;
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

        std::cerr << "tangent unclamped: " << dt *lambda << std::endl;
        double mu = 0.5;
        double mag = std::clamp(dt * lambda, -mu * impulseN.mag(), mu * impulseN.mag());
        impulseT = Vec2(t.x * mag, t.y * mag);
    }

    const Vec2 impulse = impulseN + impulseT;
    std::cerr << "contact impulse:\t" << impulse << std::endl;
    
    accumulateJ(this->b1, v1, -impulse);
    accumulateJ(this->b2, v2,  impulse);

    return impulse;
}

bool ContactConstraint::converged(const int iteration, Impulse j1, Impulse j2, double dt)
{
    auto s1 = subjectFrom(b1);
    auto s2 = subjectFrom(b2);
    subjectTo(s1, getJ(b1), dt);
    subjectTo(s2, getJ(b2), dt);

    Vec3 p1 = s1.p + v1.rotate(s1.o);
    Vec3 p2 = s2.p + v2.rotate(s2.o);
    Vec3 V1 = s1.p - v1.rotate(s1.o);
    Vec3 V2 = s2.p - v2.rotate(s2.o);
    Vec3 delP = s1.p + s1.v*dt + v1.rotate(s1.o + s1.w.z*dt) - s2.p - s1.v*dt - v2.rotate(s2.o + s2.w.z*dt);
    Vec3 delV = s1.v + (s1.w ^ Vec3(p1-s1.p)) - s2.v - (s2.w ^ Vec3(p2-s2.p));

    //injectDebugEffect(std::make_shared<VectorEffect>(Vec2(p1.x, p1.y), Vec2(V1.x, V1.y) * 0.1, Red));
    //injectDebugEffect(std::make_shared<VectorEffect>(Vec2(p2.x, p2.y), Vec2(V2.x, V2.y) * 0.1, Red));

    //std::cerr << "CONTACT: delP mag: " << delP.mag() << "\t" << "delV mag: " << delV.mag() << std::endl;

    return true;
}

// Returns impulse. Positive for penetrating body (b2)
Vec2 CoincidentConstraint::resolve (const double dt) 
{
    // https://github.com/erincatto/box2d-lite/blob/master/src/Joint.cpp#L35
    auto s1 = subjectFrom(b1);
    auto s2 = subjectFrom(b2);
    subjectTo(s1, getJ(b1), dt);
    subjectTo(s2, getJ(b2), dt);

    Body& b1 = *this->b1;
    Body& b2 = *this->b2;
    Vec3 xhat(1,0,0);
    Vec3 yhat(0,1,0);

    // Calculate Radius
    Vec3 r1 = v1.rotate(s1.o);
    Vec3 r2 = v2.rotate(s2.o);

    Vec3 delV = s1.v + (s1.w ^ r1) - s2.v - (s2.w ^ r2);
    std::cerr << "delV:\t" << delV << std::endl;

    // dV = dV0 + K * J 
    // J = - K^-1 * dV0 
    
    Mat22 Kinv;
    {
        Mat22 K1 = {
            .a = b1.getInvMass()+b2.getInvMass(), .b = 0,
            .c = 0, .d = b1.getInvMass()+b2.getInvMass()
        };
        Mat22 K2 = {
            .a = b1.getInvInertia() * r1.y * r1.y, .b =-b1.getInvInertia() * r1.y * r1.x,
            .c =-b1.getInvInertia() * r1.x * r1.y, .d = b1.getInvInertia() * r1.x * r1.x
        };
        Mat22 K3 = {
            .a = b2.getInvInertia() * r2.y * r2.y, .b =-b2.getInvInertia() * r2.y * r2.x,
            .c =-b2.getInvInertia() * r2.x * r2.y, .d = b2.getInvInertia() * r2.x * r2.x
        };
        Mat22 K = K1 + K2 + K3;
        Kinv = K.inv();
    }
    Vec2 baumgarte;
    {
        double factor = 0.1;
        double slopallowance = 1.0;

        Vec3 delP = s1.p + r1 - s2.p - r2;
        
        if (delP.mag() > slopallowance)
            baumgarte = Vec2(delP.x, delP.y) * (factor / dt); 
    }

    Vec2 impulse = Kinv * (Vec2(delV.x, delV.y) + baumgarte);

    std::cerr << "coincident impulse:\t" << impulse << std::endl;
    
    accumulateJ(this->b1, s1.p+r1, -impulse);
    accumulateJ(this->b2, s2.p+r2,  impulse);

    return impulse;
}


bool CoincidentConstraint::converged(const int iteration, Impulse j1, Impulse j2, double dt)
{
    auto s1 = subjectFrom(b1);
    auto s2 = subjectFrom(b2);
    subjectTo(s1, getJ(b1), dt);
    subjectTo(s2, getJ(b2), dt);

    Vec3 p1 = s1.p + v1.rotate(s1.o);
    Vec3 p2 = s2.p + v2.rotate(s2.o);
    Vec3 V1 = s1.p - v1.rotate(s1.o);
    Vec3 V2 = s2.p - v2.rotate(s2.o);
    Vec3 delP = s1.p + s1.v*dt + v1.rotate(s1.o + s1.w.z*dt) - s2.p - s1.v*dt - v2.rotate(s2.o + s2.w.z*dt);
    Vec3 delV = s1.v + (s1.w ^ Vec3(p1-s1.p)) - s2.v - (s2.w ^ Vec3(p2-s2.p));

    injectDebugEffect(std::make_shared<VectorEffect>(Vec2(p1.x, p1.y), Vec2(V1.x, V1.y) * 0.1, Red));
    injectDebugEffect(std::make_shared<VectorEffect>(Vec2(p2.x, p2.y), Vec2(V2.x, V2.y) * 0.1, Red));

    std::cerr << "COINCIDENT: delP mag: " << delP.mag() << "\t" << "delV mag: " << delV.mag() << std::endl;

    if (iteration >= 4) 
        return true;
    return delV.mag() < 1 && delP.mag() < 1;
    //return true;
}

// Moves b2 such that b2+v2 = b1+v1
std::shared_ptr<Constraint> CoincidentConstraint::mate (std::shared_ptr<Body> b1, Vec2 v1, std::shared_ptr<Body> b2, Vec2 v2) 
{
    Vec2 del = b1->getRel(v1) - b2->getRel(v2);
    b2->setPos(b2->getPos() + del);

    return std::make_shared<CoincidentConstraint>(v1, v2, b1, b2);
}

