#include "physicsim/world.hpp"
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

// todo broad phase detection
/*! Returns 1 when objects overlap the same coordinates in space. Currently works with only 2 bodies
 *
 * TODO: collision detection for multiple shapes, and homogenous interface
 */
bool physicsim::World::collisionDetect(RigidBody* body1, RigidBody* body2) const{
	if (body1->getType() == physicsim::Circle && body2->getType() == physicsim::Circle) {
		return this->circleCircleCollisionDetect(body1, body2);
	}
	if (body1->getType() == physicsim::Rectangle && body2->getType() == physicsim::Rectangle) {
		return rectRectCollisionDetect(body1, body2);
	}
	return 0;
}

// todo param validation on these
/*! Collision detection for circles
 */
bool physicsim::World::circleCircleCollisionDetect(RigidBody* circle1, RigidBody* circle2) const {
	return std::pow(circle1->getR() + circle2->getR(), 2) > (circle1->getPos() - circle2->getPos()).vectorMagnitudeSqrd();
}

/*! Collision detection for circle + rectangle
 */
bool physicsim::World::rectCircleCollisionDetect(RigidBody* rect, RigidBody* circle) const {
	return false;
}

/*! Collision detection for rectangles
 */
bool physicsim::World::rectRectCollisionDetect(RigidBody* rect1, RigidBody* rect2) const {
	// SAT using body1s edges first
	// get points, convert to world coords, find normal of edge
	// not sure if we have to do it for both rects
	physicsim::Matrix worldVertsB1[4];
	rect1->getVerticesWorld(worldVertsB1);
	physicsim::Matrix worldVertsB2[4];
	rect2->getVerticesWorld(worldVertsB2);


	physicsim::Matrix edge;
	physicsim::Matrix norm;
	for (int i = 0; i < 4; i++) {
		bool passed = true;
		edge = worldVertsB1[(i + 1) % 4] - worldVertsB1[i];
		norm = { 2, 1, {-1 * edge(1, 0), edge(0, 0)} };
		for (int j = 0; j < 4; j++) {
			if ((worldVertsB2[j] - worldVertsB1[i]).dot(norm) < 0) {
				passed = false;
			}
		}
		if (passed) {
			return false;
		}
	}


	return true;
}

/*! Iterate over all pairs of bodies and alter motion between each pair of bodies
 *
 * TODO: implement getMomentum() for rigidbody
 */
void physicsim::World::collisionHandler(float dt) {
	bool collision = this->collisionDetect(this->bodies[0], this->bodies[1]);
	if(collision) {
		//add impulse to each body, according to momentum and energy balance
		//impulse is force times time, and also change in momentum
		//addImpulse will turn impulse vector into force
		physicsim::Matrix i = this->bodies[1]->getMomentum() - this->bodies[0]->getMomentum();
		//TODO: getMomentum function for bodies
		this->bodies[0]->addImpulse(i, dt);
		this->bodies[1]->addImpulse((i - i) - i, dt); //bad way to write negative of i
	}
}



