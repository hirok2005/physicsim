#include "physicsim/rigidbody.hpp"
#include "physicsim/matrix.hpp"




physicsim::RigidBody::RigidBody(float x, float y, float m, float theta, physicsim::Shape type) {
	this->pos = physicsim::Matrix(2, 1); //set x and y values
	this->m = m;
	this->theta = theta;
	this->lVel = physicsim::Matrix(2, 1);
	this->aVel = 0;
	this->type = type;
}

physicsim::Shape physicsim::RigidBody::getType() {
	return this->type;
}

void physicsim::RigidBody::addImpulse(physicsim::Matrix i) {
	this->lVel = this->lVel + i.scalarDivide(this->m); //divide impulse by mass
}


physicsim::RigidRect::RigidRect(float x, float y, float m, float theta, float w, float h) : physicsim::RigidBody(x, y, m, theta, physicsim::Rectangle), vertices { physicsim::Matrix(2, 1, {-w / 2, h / 2}),
																																			 physicsim::Matrix(2, 1, {w / 2, h / 2}),
																																			 physicsim::Matrix(2, 1, {w / 2, -h / 2}),
																																			 physicsim::Matrix(2, 1, {-w / 2, -h / 2}) } {
	this->w = w; this->h = h;
}

physicsim::RigidCircle::RigidCircle(float x, float y, float m, float theta, float r) : physicsim::RigidBody(x, y, m, theta, physicsim::Circle) {
	this->r = r;
}
