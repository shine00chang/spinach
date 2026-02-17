# Spinach

A 2D physics engine made as an exercise. <br>
Written with the goal of simulating physical dynamic systems and controllers <br>
Requires SDL2, built with CMake. <br>

## The engine
The engine is built on constraint-based physics, resolved with Erin Catto’s sequential impulse.
Library API designed for building physical "environments" of "bodies," with assigned controlling functions and "positional constraints." (fixtures, or mates).

Resources:
- Erin Catto's [slide deck from GDC 2006](https://box2d.org/files/ErinCatto_SequentialImpulses_GDC2006.pdf)
- Allen Chou's [blog series on game physics](https://allenchou.net/game-physics-series/)

## Building
- Install SDL2
    - Mac: `brew install sdl2`
- Run CMake: `cmake .`
- Build: 
    - Linux: `make`

