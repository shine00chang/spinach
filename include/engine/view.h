#ifndef __VIEW_h__
#define __VIEW_h__

#include "SDL_render.h"
#include "core.h"
#include "environment.h"

#include <memory>
#include <list>

enum Colors {
    Red, 
    Blue, 
    Green,
    Black
};
void setColor ( SDL_Renderer* ren, Colors c);


class Effect {
public:
    virtual ~Effect() {};

    virtual void render(SDL_Renderer* renderer) = 0;
    virtual bool isOver() = 0;
};


class PointEffect : public Effect {
    Vec2 point;
    Colors color;
    bool persistent = false;

public:
    PointEffect(Vec2 p, Colors c, bool persistent = false) :
        point(p),
        color(c),
        persistent(persistent) {};
    ~PointEffect() {};

    void render(SDL_Renderer* renderer);
    bool isOver();
};


class VectorEffect : public Effect {
    Vec2 start;
    Vec2 end;
    Colors color;
    bool persistent = false;

public:
    VectorEffect(Vec2 start, Vec2 vec, Colors c, bool persistent = false) : 
        start(start),
        end(start + vec),
        color(c),
        persistent(persistent) {};
    ~VectorEffect() {};

    void render(SDL_Renderer* renderer);
    bool isOver();
};

// Debug Purposes. Makes it easier to inject debugging effects.
void injectDebugEffect(std::shared_ptr<Effect> effect);


class View {
    std::list<std::shared_ptr<Effect>> effects;

public:
     View() {};
    ~View() {};

    void render (SDL_Renderer* renderer, const Environment& env);
    inline void addEffect (std::shared_ptr<Effect> effect) { 
        effects.push_back( effect ); 
    };
};



#endif
