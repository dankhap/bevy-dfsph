
# bevy-dfsph
Testing Rust SPH implementation

#Log
- tested different rust graphics engiens, including:
	- wgpu: very powerfull but too low level, requires writing shaders, would like to start with something more simple and maybe later port to this
	- rapier physics engine: too high level, every object must pass throgh its built-in collision system, too limiting
	- ggez: A 2D rust game engine, looks nice and simple but is limitted to 2D and if we would like to have final 3D simulation, we will need to port the code again
	- BEVY: A new rust game engine with a nice design with support for both 2D and 3D so can start with 2D and expand later
- Decided to go with BEVY
- Different SPH methods were explored including original SPH, and divergance free SPH

#Plan
1. Simulate particles physics without collisions (DONE)
1. Add support for fast particle neighbour finding and add simple collisions

