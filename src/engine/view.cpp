#include "SDL_render.h"
#include "app.h"
#include "body.h"
#include "environment.h"
#include "view.h"

#include <vector>
#include <memory>


std::list<std::shared_ptr<Effect>> debugEffectsQueue;
void injectDebugEffect(std::shared_ptr<Effect> effect) {
    debugEffectsQueue.push_back( effect );
}


void setColor ( SDL_Renderer* ren, Colors c) {
    switch (c) {
    case Red: 
        SDL_SetRenderDrawColor( ren, 230, 0, 0, 255 );
        break;
    case Green: 
        SDL_SetRenderDrawColor( ren, 0, 230, 0, 255 );
        break;
    case Blue: 
        SDL_SetRenderDrawColor( ren, 0, 0, 230, 255 );
        break;
    case Black: 
        SDL_SetRenderDrawColor( ren, 0, 0, 0, 255 );
        break;
    }
}

void View::render(SDL_Renderer* renderer, const Environment& env) {

    // Clear renderer 
    SDL_SetRenderDrawColor( renderer, 255, 255, 255, 255 );
    SDL_RenderClear( renderer );


    // Render each body as polygon 
    for (const auto& body : env.getBodies()) 
    {
        // Set render color to grey ( rect will be rendered in this color )
        SDL_SetRenderDrawColor( renderer, body->color.r, body->color.g, body->color.b, body->color.a);

        auto points = body->getPointsGlobal();
        for (int i=0; i<points.size(); i++) {
            const Vec2& a = points[i];
            const Vec2& b = points[i == points.size()-1 ? 0 : i+1];

            SDL_RenderDrawLine( renderer, a.x, SCREEN_HEIGHT - a.y, b.x, SCREEN_HEIGHT - b.y);
        }

    }

    // DEBUG: Inject effects 
    while (!debugEffectsQueue.empty()) {
        effects.push_back( std::move(debugEffectsQueue.front()) );
        debugEffectsQueue.pop_front();
    }
    
    // Effects
    auto iter = effects.begin();
    while (iter != effects.end()) {
        (*iter)->render(renderer);
        
        if ((*iter)->isOver()) {
            iter = effects.erase(iter);
        } else {
            iter++;
        }
    }

    // Actually Render
    SDL_RenderPresent( renderer );
}
