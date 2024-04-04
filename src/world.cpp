#include "physicsim/world.hpp"
#include "physicsim/constants.hpp"
#include <iostream>
// world is in centimetres

/*! Constructs world with dimensions x, y
 */
physicsim::World::World(int x, int y) : X(x), Y(y) {}

/*! Adds a rigidbody to world. This lets world manage the body
 */
void physicsim::World::addBody(RigidBody* body) {
	this->bodies.push_back(body);
	std::cout << "f" << std::endl;
}

/*! Function to update all bodies in the world. Runs with the given delta time for all suvatesque equations
 */
void physicsim::World::step(float dt) {
	for (int i = 0; i < this->bodies.size(); i++) {
		this->bodies[i]->update(dt);
	}
}
