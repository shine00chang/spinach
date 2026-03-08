#include "constraint.h"
#include "environment.h"
#include "app.h"
#include <SDL_keycode.h>
#include <memory>

/* Falling Diamond
 */ 
Environment falling() {
    Environment env;

    auto r1 = Body::makeRect(340, 300, 50, 50, 10);
    r1->setOrient(std::atan(0.8));
    auto floor = Body::makeRect(340, 0, 680, 50, 0);
    floor->setGravity(false);

    env.addBody(r1);
    env.addBody(floor);

    return env;
}

/* Stacking
 */ 
Environment stacking() {
    Environment env;

    auto r1 = Body::makeRect(340, 300, 50, 50, 10);
    auto r2 = Body::makeRect(340, 200, 50, 50, 10);
    auto r3 = Body::makeRect(340, 100, 50, 50, 10);

    auto floor = Body::makeRect(340, 0, 680, 50, 0);
    floor->setGravity(false);

    env.addBody(r1);
    env.addBody(r2);
    env.addBody(r3);
    env.addBody(floor);

    return env;
}

/* Gravity & Floor. 
 * Creates random falling block when space bar is pressed
 */
void rainController (Environment* env, const Application& app, View& view, double dt) {
    if (!app.isPressed(SDLK_SPACE))
        return;

    double x = app.mouse().x;
    double y = app.mouse().y;
    double s = 60; //std::rand() % 50 + 20;
    double ang = std::asin(std::rand() % 1000 / 1000.0);

    auto b = Body::makeRect(x, y, s, s, 10);
    b->setOrient(ang);
    env->addBody(b);
}

Environment rain () {
    Environment env;

    auto r1 = Body::makeRect(340, 300, 50, 50, 10);
    r1->setOrient(std::atan(0.8));
    auto floor = Body::makeRect(340, 50, 500, 50, 0);
    floor->setGravity(false);

    env.addBody(floor);
    env.addController(rainController);

    return env;
}

void cartControllerManual (Body* body, const Application& app, View& view, double dt) {
    static double v = 0;
    if (app.isPressed(SDLK_LEFT)) {
        v -= 20;
        body->setVelo(Vec2(v, 0));
    }
    if (app.isPressed(SDLK_RIGHT)) {
        v += 20;
        body->setVelo(Vec2(v,0));
    }
    body->setVelo(Vec2(v, 0));
}

Environment physicalPendulum () {
    Environment env;
    auto cart = Body::makeRect(340, 400, 100, 30, 0);
    auto pend = Body::makeRect(0, 0, 200, 200, 10);
    
    cart->setGravity(false);
    cart->useController(cartControllerManual);

    pend->setOrient(3.14 / 4.0);

    auto mate = CoincidentConstraint::mate(cart, Vec2(0, -15), pend, Vec2(100, 100)); 
    std::cout << pend->getInvInertia() << std::endl;

    env.addBody(cart);
    env.addBody(pend);
    env.addConstraint(mate);
    return env;
}

Environment pendulum () {
    Environment env;
    auto cart = Body::makeRect(340, 400, 100, 30, 0);
    auto pend = Body::makeRect(0, 0, 20, 20, 10);
    
    cart->setGravity(false);
    cart->useController(cartControllerManual);

    pend->setOrient(3.14 / 4.0);
    std::cout << pend->getInvInertia() << std::endl;

    auto mate = CoincidentConstraint::mate(cart, Vec2(0, -15), pend, Vec2(100, 100)); 

    env.addBody(cart);
    env.addBody(pend);
    env.addConstraint(mate);
    return env;
}

double thetaZero = -100;
std::shared_ptr<Body> pend;
int counter = 0;

void invertedPendulumController (Body* body, const Application& app, View& view, double dt) {
    if (counter++ < 10) {
        return;
    }
    counter = 0;

    double theta = pend->getOrientation();
    if (thetaZero == -100) thetaZero = theta;
    theta -= thetaZero;
    static double thetaPrev = theta;

    //double kP = theta * thetaPrev < 0 ? 10 : 30;
    double kP = 0;
    double kI = 10000;
    double kD = theta * thetaPrev < 0 ? 5 : 10;
    static double I = 0;
    static double D = 0;
    
    I += theta * dt;
    D = (theta - thetaPrev) / dt;
    thetaPrev = theta;
    
    double v = kP * theta + kI * I + kD * D;

    std::cout << theta << "\t" << I << "\t" << D << "\tout: " << v << std::endl;

    body->setVelo(Vec2(-v, 0));
}

void invertedPendulumDisturbanceController (Body* body, const Application& app, View& view, double dt) {
    double F = 10000;
    if (app.isPressed(SDLK_LEFT)) {
        body->applyForce(Vec2(-F, 0));
    }
    if (app.isPressed(SDLK_RIGHT)) {
        body->applyForce(Vec2(F,0));
    }
}

Environment invertedPendulum () {
    Environment env;
    auto cart = Body::makeRect(340, 100, 100, 30, 0);
    pend = Body::makeRect(0, 0, 100, 100, 10);
    
    cart->setGravity(false);
    cart->useController(invertedPendulumController);
    pend->useController(invertedPendulumDisturbanceController);

    pend->setOrient(3.14 / 4.0);

    auto mate = CoincidentConstraint::mate(cart, Vec2(0, 15), pend, Vec2(-50, -50)); 

    env.addBody(cart);
    env.addBody(pend);
    env.addConstraint(mate);
    return env;
}

void setEnvs () {
    DefaultEnv = "invertedPendulum";
    EnvironmentLibrary = {
        {"falling", falling()},
        {"rain", rain()},
        {"stacking", stacking()},
        {"physicalPendulum", physicalPendulum()},
        {"pendulum", pendulum()},
        {"invertedPendulum", invertedPendulum()}
    };
}
