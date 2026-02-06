#include "environment.h"
#include "app.h"
#include <SDL_keycode.h>

/*
 * Debug / Test purposes. 
 * WASD to increment position. Arrow keys to apply force.
 */
void debugController (Body* body, const Application& app, View& view) {
    if (app.isHeld(SDLK_w)) {
        body->applyForce(Vec2(0, 100));
    }
    if (app.isHeld(SDLK_a)) {
        body->applyForce(Vec2(-100, 0));
    }
    if (app.isHeld(SDLK_s)) {
        body->applyForce(Vec2(0, -100));
    }
    if (app.isHeld(SDLK_d)) {
        body->applyForce(Vec2(100, 0));
    }
    if (app.isHeld(SDLK_q)) {
        body->setAngAccl( body->getAngAccl() + 0.1 );
    }
    if (app.isHeld(SDLK_e)) {
        body->setAngAccl( body->getAngAccl() - 0.1 );
    }
}

/*
 * A floor and two blocks, one with some downward initial velocity
 * This tests for gravity, basic collision, and sinking.
 */ 
Environment test1() {
    Environment env; 

    auto r1 = Body::makeRect(360, 400, 50, 50, 10);
    auto r2 = Body::makeRect(300, 330, 50, 50, 10);
    r1->applyForce(Vec2{0, -10000});
    r1->setOrient(std::atan(0.6));
    r2->setOrient(std::atan(1.5));

    auto floor = Body::makeRect(340, 0, 680, 50, 1e10);
    floor->setGravity(false);

    env.addBody(r1);
    env.addBody(r2);
    env.addBody(floor);

    return env;
}

/*
 * Two blocks, one with a force controller, and no gravity.
 * Tests for collision detection & resolution.
 */ 
Environment test2() {
    Environment env;

    auto r1 = Body::makeRect(340, 300, 50, 50, 10);
    auto r2 = Body::makeRect(300, 200, 50, 50, 10);

    r1->setGravity(false);
    r2->setGravity(false);

    r1->useController(debugController);

    env.addBody(r1);
    env.addBody(r2);


    return env;
}

/*
 * A block and a diamond, one with a force controller, and no gravity.
 * Tests for collision detection on a non-AABB shape.
 */ 
Environment test3() {
    Environment env;

    auto r1 = Body::makeDiamond(300, 300, 50, 10);
    auto r2 = Body::makeRect(200, 200, 50, 50, 10); 

    r1->setGravity(false);
    r2->setGravity(false);

    r1->useController(debugController);

    env.addBody(r1);
    env.addBody(r2);


    return env;
}

/* Falling Diamond
 */ 
Environment fallingDiamond() {
    Environment env;

    auto r1 = Body::makeRect(340, 300, 50, 50, 10);
    r1->setOrient(std::atan(0.8));
    auto floor = Body::makeRect(340, 0, 680, 50, 1e10);
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

    auto floor = Body::makeRect(340, 0, 680, 50, 1e10);
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
void rainyController (Environment* env, const Application& app, View& view) {
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

Environment rainyDay () {
    Environment env;

    auto r1 = Body::makeRect(340, 300, 50, 50, 10);
    r1->setOrient(std::atan(0.8));
    auto floor = Body::makeRect(340, 50, 500, 50, 1e10);
    floor->setGravity(false);

    env.addBody(floor);
    env.addController(rainyController);

    return env;
}

void setEnvs () {
    DefaultEnv = "rainyDay";
    EnvironmentLibrary = {
        {"test1", test1()},
        {"test2", test2()},
        {"test3", test3()},
        {"fallingDiamond", fallingDiamond()},
        {"rainyDay", rainyDay()},
        {"stacking", stacking()},
    };
}
