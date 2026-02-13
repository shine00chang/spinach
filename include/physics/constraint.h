#ifndef __CONSTRAINT_h__
#define __CONSTRAINT_h__

#include "core.h"
#include <memory>

class Body;
class Constraint {
public:
    std::shared_ptr<Body> b1;
    std::shared_ptr<Body> b2;
    Vec2 v1;
    Vec2 v2;

    Constraint (std::shared_ptr<Body> b1, std::shared_ptr<Body> b2, const Vec2& v1, const Vec2& v2) : 
        b1(b1), b2(b2), v1(v1), v2(v2) {};

    virtual Vec2 resolve(const double dt) =0;
    virtual bool converged(const int iteration) =0;

    virtual ~Constraint() {};
};

class ContactConstraint : public Constraint {
public:

    Vec2 norm;

    ContactConstraint (const Vec2& norm, const Vec2& v1, const Vec2& v2, std::shared_ptr<Body> b1, std::shared_ptr<Body> b2) :
        Constraint(b1, b2, v1, v2), norm(norm) {};
    Vec2 resolve(const double dt) override;
    bool converged(const int iteration) override;

    ~ContactConstraint() override {}
};

#endif
