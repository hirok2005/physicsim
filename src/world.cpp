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

// todo broad phase detection
bool physicsim::World::collisionDetect(RigidBody* body1, RigidBody* body2) const{
	if (body1->getType() == physicsim::Circle && body2->getType() == physicsim::Circle) {
		return std::pow(body1->getR() + body2->getR(), 2) > (body1->getPos() - body2->getPos()).vectorMagnitudeSqrd();
	}
}



