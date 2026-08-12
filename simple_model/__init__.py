"""Ultra-simple, closed-form pulsejet/ramjet/point-mass sizing model.

Deliberately separate from ``src/douglas_dart`` -- a handful of dimensional
inputs (vehicle diameter, throat diameter, chamber length, throat length,
wingspan, fuel type) swept through closed-form thrust/drag equations, no
ODE integration inside any single physics call. Only reuses
``douglas_dart.atmosphere`` (a pure, dependency-free standard-atmosphere
function) from the main package -- everything else here is self-contained.
"""
