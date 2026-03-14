#include "app.h"
#include "environment.h"
#include "view.h"
#include <cmath>

/* Gravity & Floor. 
 * Creates random falling block when space bar is pressed
 */
constexpr double k_launchx = 50;
constexpr double k_launchy = 200;
constexpr double k_launchForce = 500;

std::shared_ptr<Body> bird;

void envController (Environment* env, const Application& app, View& view, double dt) 
{
    // Cursor
    {
        auto effect = std::make_shared<PointEffect>(Vec2(app.mouse().x, app.mouse().y), Black);
        view.addEffect(effect);
    }
    // Launch Bird
    if (app.isPressed(SDLK_SPACE)) {
        bird->setGravity(true);
        bird->setVelo(Vec2(std::sqrt(3)/2.0, 0.5) * k_launchForce);
    }
    // Spawn in new block
    if (app.mouseClicked()) {
        
        double x = app.mouse().x;
        double y = app.mouse().y;

        auto b = Body::makeRect(x, y, 50, 50, 10);

        env->addBody(b);
    }
}


Environment env () {
    Environment env;

    auto sling = Body::makeRect(50, 25 + 50, 20, 100, 1e-10);
    bird = Body::makeRect(k_launchx, k_launchy, 30, 30, 100);
    bird->setGravity(false);
    auto floor = Body::makeRect(340, 0, 680, 50, 1e10);
    floor->setGravity(false);

    env.addBody(floor);
    env.addBody(sling);
    env.addBody(bird);
    env.addController(envController);

    return env;
}


void setEnvs () {
    DefaultEnv = "angrybirds";
    EnvironmentLibrary = {
        {"angrybirds", env()}
    };
}
