#include "environment.h"
#include "view.h"

std::map<std::string, Environment> EnvironmentLibrary;
std::string DefaultEnv;

void Environment::runControllers (Application& app, View& view) {
    for (auto& c : m_controllers)
        c(this, app, view);
}
