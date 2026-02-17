#ifndef __ENVIRONMENT_h__
#define __ENVIRONMENT_h__

#include "controller.h"
#include "body.h"

#include <memory>
#include <vector>
#include <map>
#include <string>


constexpr double GRAVITY = -40;

class Application;
class View;
class Environment {
    Bodies m_bodies;
    std::vector<EnvController> m_controllers;


public: 
     Environment() {};
    ~Environment() {};

    void runControllers (Application& app, View& view);

    void addBody       (const std::shared_ptr<Body> body) { m_bodies.push_back(body); } 
    void addController (const EnvController controller) { m_controllers.push_back(controller); }

    Bodies& getBodiesMut() { return m_bodies; }
    const Bodies& getBodies() const { return m_bodies; }
};

extern std::map<std::string, Environment> EnvironmentLibrary;
extern std::string DefaultEnv;

#endif
