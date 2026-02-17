#ifndef __APP_h__
#define __APP_h__

#include "SDL.h"
#include "SDL_keycode.h"
#include "environment.h"
#include "view.h"

#include <memory>
#include <set>


const int SCREEN_WIDTH = 640;
const int SCREEN_HEIGHT = 480;

struct MousePos {
    int32_t x;
    int32_t y;
};

class Application {

    SDL_Window* m_window;
    SDL_Renderer* m_renderer;

    // Event Variables
    std::set<SDL_Keycode> m_keys;
    std::set<SDL_Keycode> m_keysHeld;

    bool m_mouseClick = false;
    MousePos m_mouse{0, 0};

    bool m_running = true;
    bool m_paused = false;

    void loop (View view, Environment env);
    void updateEvents ();
public: 


     Application();
    ~Application();

    void start(Environment env);

    bool mouseClicked () const { return m_mouseClick; }
    MousePos mouse    () const { return m_mouse; }

    inline bool isPressed (SDL_Keycode k) const { return m_keys.count(k); }
    inline bool isHeld    (SDL_Keycode k) const { return m_keysHeld.count(k); }
    inline bool isRunning () const { return m_running; }
    inline bool isPaused () { return m_paused; }
    inline void pause () { m_paused = true; }
    inline void resume () { m_paused = false; }
};

#endif
