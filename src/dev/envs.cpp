#include "constraint.h"
#include "environment.h"
#include "app.h"
#include <SDL_keycode.h>
#include <memory>

/* Stacking
 */ 
Environment stacking() {
    Environment env;

    auto floor = Body::makeRect(340, 50, 500, 50, 0);
    floor->setGravity(false);
    env.addBody(floor);

    // Builds a pyramid
    int N = 3;
    for (int i=0; i<N; i++) {
        for (int j=0; j<N-i; j++) {
            int x = -(N-i) * 55 / 2 + j * 55;
            int y = i * 50;
            auto r = Body::makeRect(340+x, 100+y, 50, 50, 10);
            env.addBody(r);
        }
    }
    
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

void setEnvs () {
    DefaultEnv = "invertedPendulum";
    EnvironmentLibrary = {
        {"rain", rain()},
        {"stacking", stacking()},
    };
}
