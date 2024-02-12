#include "physicsim/rigidbody.hpp"
#include "physicsim/matrix.hpp"

physicsim::RigidBody::RigidBody(float x, float y, float m, float theta) {
	this->pos = physicsim::Matrix(2, 1); //set x and y values
	this->m = m;
	this->theta = theta;
	this->vel = physicsim::Matrix(2, 1);
}

void physicsim::RigidBody::addImpulse(physicsim::Matrix i) {
	this->vel = this->vel + i.scalarDivide(this->m); //divide impulse by mass
}
