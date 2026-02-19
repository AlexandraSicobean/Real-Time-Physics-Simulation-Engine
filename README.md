# Real-Time Physics Simulation Engine

This project was developed as part of a Computer Animation course and represents a physics simulation framework including numerical integration methods, particle systems, cloth simulation, and Smoothed Particle Hydrodynamics (SPH).

## Project Overview
The project implements simulations for:
- Particle systems
- Spring-mass systems
- Cloth behavior (Provot model)
- Fluid simulation (SPH)
- Collision detection and response
- Multiple numerical integrators

## Demo Video
[Watch demo video here](https://www.youtube.com/watch?v=yD6drU-uNPE)

## Numerical Integrators

Implemented:
- Explicit Euler
- Midpoint
- RK2
- RK4
- Symplectic Euler
- Verlet

These integrators were used throughout the rest of the simulations.
The project compares stability and behavior of different integration schemes.

## Force Models

Implemented:
- Constant acceleration (gravity)
- Linear & quadratic drag
- Spring forces

Force composition supports combining multiple force types.

## Particle Systems & Collisions

Implemented:
- SceneProjectiles
- SceneRope
- SceneFountain
- SceneNBody

Collision detection with:
- Plane
- Sphere
- AABB

Features:
- Particle-particle collision
- Mouse interaction with colliders
- Dynamic integrator switching
- UI parameter control

Challenges included stable collision response and correct normal orientation.

## Cloth Simulation (Provot’s Model)

Implemented:
- Provot spring model
- Structural springs
- Collision handling
- Interaction:
  - Grab & move particles
  - Fix/release points

Key aspects:
- Spring relaxation steps
- Stretch control
- Stability tuning

Self-intersections were not implemented.
The scene includes a cloth interacting with a sphere inside an enclosed environment.

## Smoothed Particle Hydrodynamics (SPH)

Implemented:
- Density computation
- Pressure computation
- Acceleration computation
- Time integration

Challenges:
- Parameter tuning (viscosity, timestep, speed of sound)
- Stability issues
- Lack of neighbor-based optimization (brute-force approach)

The SPH implementation demonstrates the fundamentals of fluid simulation using particle-based methods. However, further parameter tuning is needed to ensure simulation stability.
