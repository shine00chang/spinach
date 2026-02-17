#ifndef __CONSTRAINT_h__
#define __CONSTRAINT_h__

#include "core.h"
#include <memory>
#include <list>

struct Impulse {
    Vec2 J;
    double L;
};
class Body;
class Environment;
class Constraint {
public:
    std::shared_ptr<Body> b1;
    std::shared_ptr<Body> b2;

    Constraint (std::shared_ptr<Body> b1, std::shared_ptr<Body> b2) : 
        b1(b1), b2(b2) {};

    virtual Vec2 resolve(const double dt) =0;
    virtual bool converged(const int iteration, Impulse j1, Impulse j2) =0;

    virtual ~Constraint() {};
};
typedef std::list<std::shared_ptr<Constraint>> Constraints;

// C: Jnorm * V >= 0
// That is, the relative velocity along the contact normal is GE 0.
// This constraint handles collisions, for stability and physical similarity sake, it implements many other corrections
// such as friction, Baumgarte stability, restitution, and slop.
class ContactConstraint : public Constraint {
public:
    // NOTE: by convention, contact edge belongs to b1, that is, norm points away from b1.
    // b2 is the penetrating body.
    Vec2 norm;

    // NOTE: points are in absolute
    Vec2 v1;
    Vec2 v2;

    ContactConstraint (const Vec2& norm, const Vec2& v1, const Vec2& v2, std::shared_ptr<Body> b1, std::shared_ptr<Body> b2) :
        Constraint(b1, b2), norm(norm), v1(v1), v2(v2) {};
    Vec2 resolve(const double dt) override;
    bool converged(const int iteration, Impulse j1, Impulse j2) override;

    ~ContactConstraint() override {}
};

// C: J * V == 0, where J is on all axis.
// This enforces two verticies on two bodies to be coincident.
class CoincidentConstraint : public Constraint {
public:
    // NOTE: points are in absolute.
    Vec2 v1;
    Vec2 v2;

    CoincidentConstraint (const Vec2& v1, const Vec2& v2, std::shared_ptr<Body> b1, std::shared_ptr<Body> b2) :
        Constraint(b1, b2), v1(v1), v2(v2) {};
    Vec2 resolve(const double dt) override;
    bool converged(const int iteration, Impulse j1, Impulse j2) override;

    ~CoincidentConstraint() override {}
};

Constraints collide (Environment& env, const double dt);
void sequentialImpulse (const Constraints constraints, const double dt);

#endif
