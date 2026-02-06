#include "environment.h"
#include "app.h"
#include <SDL_keycode.h>


Environment test() {
    Environment env;

    auto r1 = Body::makeRect(300, 300, 50, 10, 10);
    auto r2 = Body::makeRect(320, 300, 50, 50, 10); 

    r1->setGravity(false);
    r2->setGravity(false);

    r1->setOrient(std::atan(0.2));

    env.addBody(r1);
    env.addBody(r2);

    return env;
}


void setEnvs () {
    DefaultEnv = "test";
    EnvironmentLibrary = {
        {"test", test ()}
    };
}
