#include "physicsim/world.hpp"
// world is in centimetres


physicsim::World::World(int x, int y) : X(x), Y(y) {}

void physicsim::World::addBody(RigidBody* body) {
	this->bodies.push_back(body);
}

void physicsim::World::step(float dt) {
	for (int i = 0; i < this->bodies.size(); i++) {
		this->bodies[i]->update(dt);
	}
}



