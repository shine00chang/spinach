#include "app.h"
#include "environment.h"
#include "view.h"

#include <memory>
#include <ostream>


/* Gravity & Floor. 
 * Creates random falling block when space bar is pressed
 */
bool launching = false;
bool launched = false;
constexpr double k_launchx = 50;
constexpr double k_launchy = 200;
constexpr double k_launchForce = 1e6;

std::shared_ptr<Body> bird;

void envController (Environment* env, const Application& app, View& view, double dt) 
{
    // Cursor
    {
        auto effect = std::make_shared<PointEffect>(Vec2(app.mouse().x, app.mouse().y), launching ? Red : Black);
        view.addEffect(effect);
    }
    // Launch Bird
    if (app.isPressed(SDLK_SPACE)) {
        launching = !launching;
    }
    // Spawn in new block
    if (app.mouseClicked()) {
        
        if (!launching) {
            double x = app.mouse().x;
            double y = app.mouse().y;

            auto b = Body::makeRect(x, y, 50, 50, 10);

            env->addBody(b);
        }
        else 
        {
            double r = M_PI + 
                (k_launchx > app.mouse().x ? -M_PI : 0) +
                std::atan((app.mouse().y - k_launchy) / (app.mouse().x - k_launchx));
                    
            Vec2 vec = Vec2(std::cos(r), std::sin(r));
            bird->setGravity(true);
            bird->applyForce( vec * k_launchForce);
        }
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
