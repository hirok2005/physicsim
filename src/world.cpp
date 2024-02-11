#include "physicsim/world.hpp"

physicsim::World::World(int x, int y) : X(x), Y(y) {}

void physicsim::World::addBody(RigidBody* body) {
	this->bodies.push_back(body);
}


