#include "app.h"
#include "view.h"

void PointEffect::render(SDL_Renderer* renderer) {

    setColor(renderer, color);
    SDL_Rect r;
    r.x = point.x-2;
    r.y = SCREEN_HEIGHT - (point.y-2);
    r.w = 4;
    r.h = 4; 

    SDL_RenderFillRect( renderer, &r );
}

bool PointEffect::isOver() {
    return !persistent;
}

void VectorEffect::render(SDL_Renderer* renderer) {

    setColor(renderer, Black);
    SDL_Rect r;
    r.x = start.x-2;
    r.y = SCREEN_HEIGHT - (start.y-2);
    r.w = 4;
    r.h = 4; 

    SDL_RenderFillRect( renderer, &r );
    
    setColor(renderer, color);
    SDL_RenderDrawLine( renderer, start.x, SCREEN_HEIGHT - start.y, end.x, SCREEN_HEIGHT - end.y );
}

bool VectorEffect::isOver() {
    return !persistent;
}
