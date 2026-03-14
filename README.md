# Spinach

A 2D physics engine made as an exercise. <br>
Written with the goal of simulating physical dynamic systems and controllers. <br>
Uses SDL2, built with CMake. <br>

## Engine
The engine is built on constraint-based physics, resolved with sequential impulse as explained by [Erin Catto](https://box2d.org/files/ErinCatto_SequentialImpulses_GDC2006.pdf) and [Allen Chou](https://allenchou.net/game-physics-series/).
Engine API designed for building physical "environments" of "bodies," with assigned controlling functions and "constraints" (coincidence, or mates).

Note:
debug logs are written to stderr. To remove these logs from console, use `... 2>/dev/null`.
`p` pauses the simulation. The simulation is paused by default, press `p` to start.
`n` continues the simulation by one frame. Accessible only in the paused state.

## Examples
The `./bin/dev <env-name>` binary builds several environments for testing and demonstration. <br>
`dev rainy` showcases gravity, collision, and stacking. Press space to spawn a block at the mouse position. <br>
`dev stacking` showcases stacking. This is an important metric for solver stability. Spinach is not too good at this. <br>

The `./bin/pendulum <env-name>` binary builds another set of environments, pertaining to pendulums of course. <br>
`pendulum simple` showcases the coincidence constraint with a pendulum on a cart, Press left and right arrow to move. <br>
`pendulum physical` showcases the coincidence constraint with a pendulum on a cart, Press left and right arrow to move. <br>
`pendulum inverted` showcases a controller atop Spinach. Press left and right arrow to add disturbance to the pendulum. <br>

The `./bin/angrybirds` binary builds a simple angry bird game. Left click spawns in blocks at the mouse. Press space to shoot<br>


## Building
- Install SDL2
    - Mac: `brew install sdl2`
- Run CMake: `cmake .`
- Build: `make`
- Run: `./bin/dev`

