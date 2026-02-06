#include "app.h"
#include "SDL.h"
#include "SDL_events.h"
#include "SDL_render.h"
#include "environment.h"
#include "SDL_timer.h"
#include "SDL_keyboard.h"
#include "environment.h"

#include <string>
#include <format>
#include <memory>


Application::Application () {
	//Initialize SDL
	if( SDL_Init( SDL_INIT_VIDEO ) < 0 ) 
		throw "SDL could not initialize!\nSDL_Error: {}\n"; // SDL_GetError();

    //Create window
    m_window = SDL_CreateWindow( "SDL Tutorial", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN );
    if( m_window == NULL ) 

        throw "Window could not be created! SDL_Error: %s\n"; // SDL_GetError();
                                                              //
    // Setup renderer 
    m_renderer = SDL_CreateRenderer( m_window, -1, SDL_RENDERER_ACCELERATED);
}

Application::~Application () {
    // Destroy window
	SDL_DestroyWindow( m_window );

    //Quit SDL subsystems
	SDL_Quit();
}

// Check & Update state according to events.
void Application::updateEvents() {
    // clear keys. keys should only include keys that were just pressed 
    m_keys.clear();
    m_mouseClick = false;

    SDL_Event ev;
    while (SDL_PollEvent(&ev) != 0) {
        switch( ev.type ){
        
        // If app quit
        case SDL_QUIT:
            m_running = false;
            break;

        // Update keys
        case SDL_KEYDOWN:
            m_keysHeld.insert(ev.key.keysym.sym);
            m_keys.insert(ev.key.keysym.sym);
            break;

        case SDL_KEYUP:
            m_keysHeld.erase(ev.key.keysym.sym);
            break;

        // Mouse
        case SDL_MOUSEBUTTONDOWN: 
            m_mouseClick = true;
            break;

        case SDL_MOUSEMOTION:
            m_mouse.x = ev.motion.x;
            m_mouse.y = SCREEN_HEIGHT - ev.motion.y;
            break;
        }
    }
}


bool paused = true;
int frame_number = 0;

void Application::loop (View view, Environment env) 
{
    constexpr double dt = 0.016;
    uint64_t prevIterMs = SDL_GetTicks64();

    while (isRunning()) 
    {
        // Get Dt
        const uint64_t ms = SDL_GetTicks64(); 
        // Throttle to DT between frames
        if (ms - prevIterMs < dt * 1000) continue;
        prevIterMs = ms;

        // Check quit event
        updateEvents();

        // Pause toggle
        if (isPressed(SDLK_p)) paused = !paused;

        // If paused and next frame not pressed, skip.
        if (paused && !isPressed(SDLK_RIGHT)) {
            //view.render(m_renderer, env);
            continue;
        }

        // Frame counter
        frame_number ++;

        // Movement
        env.runControllers(*this, view);

        for (auto& body : env.getBodiesMut()) 
        {
            // Controllers
            body->runControllers(*this, view);

            // Gravity
            if (body->getGravity())
                body->applyForce(Vec2(0, GRAVITY * body->getMass()));

            // Integration
            body->accumulateForces(dt);
        }

        // Update
        for (auto& body : env.getBodiesMut()) 
        {
            body->update(dt);
        }

        // Collision & Correction
        env.collide(dt);

        // Render
        view.render(m_renderer, env);
    }
}

void Application::start (Environment env)
{
    View view = View();
    loop(std::move(view), std::move(env));
}
