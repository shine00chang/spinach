#ifndef __BODY_h__
#define __BODY_h__

#include "SDL.h"
#include "constraint.h"
#include "core.h"
#include "controller.h"

#include <vector>
#include <memory>

class Application;
class View;
class Body 
{
    bool gravity = true;

    double mass;
    double invMass;
    double inertia;
    double invInertia;
    double friction = 0.5;

    std::vector<Vec2> points;

    Vec2 pos;
    Vec2 velo{0, 0};
    Vec2 accl{0, 0};

    double orient = 0;
    double angVelo = 0;
    double angAccl = 0;

    std::vector<Controller> m_controllers;

public: 
    SDL_Color color {0,0,0}; 

     Body(double x, double y, std::vector<Vec2> points, double mass);
    ~Body() {};

    void useController  (Controller controller) { m_controllers.push_back(controller); }
    void runControllers (const Application& app, View& view); 

    void applyForce (const Vec2& f);
    void accumulateForces (const double dt);
    void update (const double dt);

    // Getters
    inline double getFriction () const { return friction; }
    inline double getMass  () const { return mass; }
    inline double getInvMass  () const { return invMass; }
    inline double getInvInertia  () const { return invInertia; }
    inline bool getGravity () const { return gravity; }
    inline Vec2 getPos     () const { return pos; }
    inline Vec2 getVelo    () const { return velo; }
    inline double getAngVelo () const { return angVelo; }
    inline double getAngAccl () const { return angAccl; }

    // Point getters, has transformation logic 
    const std::vector<Vec2>& getPointsRaw  () const { return points; }
    const std::vector<Vec2> getPointsLocal () const { 
        auto v = points;
        for (Vec2& p : v) 
            p = p.rotate(orient);
        return v;
    }
    const std::vector<Vec2> getPointsGlobal () const { 
        auto v = points;
        for (Vec2& p : v) 
            p = p.rotate(orient) + pos;
        return v;
    }
    
    // Setter
    inline void setGravity (bool b)        { gravity = b; }
    inline void setPos     (const Vec2& v) { pos = v; }
    inline void setVelo    (const Vec2& v) { velo = v; } 
    inline void setAngAccl (double a)      { angAccl = a; }
    inline void setOrient  (double o)      { orient = o; }

    // Mutation
    // Apply impulse J and angular impulse L
    void impulse(const Vec2 J, const double L);

    // Convenience factories
    static std::shared_ptr<Body> makeRect(double x, double y, double w, double h, double m);
    static std::shared_ptr<Body> makeDiamond(double x, double y, double r, double m);
};

typedef std::vector<std::shared_ptr<Body>> Bodies;
#endif
