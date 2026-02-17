#ifndef __CONTROLLER_h__
#define __CONTROLLER_h__

class Body;
class Environment;
class Application;
class View;

using Controller = void (*)(Body* body, const Application& app, View& view);
using EnvController = void (*)(Environment* env, const Application& app, View& view);

#endif
