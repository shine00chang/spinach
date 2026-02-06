#include "body.h"
#include "app.h"
#include <memory>



Body::Body(double x, double y, std::vector<Vec2> points, double mass) : 
     mass(mass), 
     points(points), 
     pos(x, y)
{    
    // Must be a polygon
    if (points.size() == 3) throw "Not a polygon";

    // if mass = 0, will do infinite mass
    if (mass == 0) {
        mass = 0;
        invMass = 0;
    } else {
        invMass = 1 / mass;
    }

    // Inertia 
    double r = 0;
    for (const auto& p : points) 
        r += p.mag();
    r /= points.size();

    inertia = mass * r * r * 0.5;
    if (mass == 0) {
        invInertia = 0;
    } else { 
        invInertia = 1.0 / inertia;
    }
};


// Apply force
void Body::applyForce(const Vec2 &f) {
    accl = accl + f * invMass;
}


// Accumulate Force
void Body::accumulateForces(const double dt) {
    velo = velo + accl * dt;
    angVelo += angAccl * dt;
}

void Body::impulse(const Vec2 j, const Vec2 r) {
    velo = velo + (j * invMass);
    angVelo += invInertia * (Vec2(-r.y, r.x) * j);
}

// Integration
/* Integrates acceleration and velocity, and clears acceleration */
void Body::update(const double dt) {

    std::cout << "velocity, angvelocity: " << velo << ", " << angVelo << std::endl;
    pos = pos + velo * dt;
    orient += angVelo * dt;

    // Reset Accel
    accl = Vec2{0, 0};
    angAccl = 0;
}


// Have controllers act on object. 
void Body::runControllers(const Application &app, View& view) {
    for (auto c : m_controllers) 
        c(this, app, view);
}

// Convenience Factory function
std::shared_ptr<Body> Body::makeRect(double x, double y, double w, double h, double m) {
    std::vector<Vec2> v{
        Vec2(-w/2, -h/2),
        Vec2( w/2, -h/2),
        Vec2( w/2,  h/2),
        Vec2(-w/2,  h/2),
    };
    std::shared_ptr<Body> ptr( new Body(x, y, v, m) );

    return ptr;
}

// Convenience Factory function
std::shared_ptr<Body> Body::makeDiamond(double x, double y, double r, double m) {
    std::vector<Vec2> v{
        Vec2( 0, -r),
        Vec2( r,  0),
        Vec2( 0,  r),
        Vec2(-r,  0),
    };
    std::shared_ptr<Body> ptr( new Body(x, y, v, m) );

    return ptr;
}


