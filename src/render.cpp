#include "physicsim/render.hpp"
#include "physicsim/world.hpp"

physicsim::Renderer::Renderer(World* sim) {
	this->sim = sim;
	this->width = sim->X * 2; this->height = sim->Y * 2;
	this->window.create(sf::VideoMode(this->width, this->height), "Render");
}

void physicsim::Renderer::update() {
	for (physicsim::RigidBody *body : this->sim->bodies) {
		if (body->getType() == physicsim::Rectangle) {

		}
	}
}
