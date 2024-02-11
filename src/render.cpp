#include "physicsim/render.hpp"
#include "physicsim/world.hpp"

physicsim::Renderer::Renderer(World* sim) {
	this->sim = sim;
	this->width = sim->X; this->height = sim->Y;
}
