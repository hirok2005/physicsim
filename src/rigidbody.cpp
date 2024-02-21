#include "physicsim/rigidbody.hpp"
#include "physicsim/matrix.hpp"
#include <stdexcept>



physicsim::RigidBody::RigidBody(float x, float y, float m, float theta, physicsim::Shape type) {
	this->pos = physicsim::Matrix(2, 1, {x, y}); //set x and y values
	this->m = m;
	this->theta = theta;
	this->lVel = physicsim::Matrix(2, 1);
	this->aVel = 0;
	this->type = type;
}

physicsim::RigidBody::RigidBody(float x, float y, float m, float theta, float w, float h)
	: vertices{ { 2, 1, {-w / 2, h / 2} }, { 2, 1, { w / 2, h / 2 } }, { 2, 1, { w / 2, -h / 2 } }, { 2, 1, { -w / 2, -h / 2 } } },
	pos(2, 1, { x, y }),
	m(m),
	theta(theta),
	lVel(2, 1),
	aVel(0),
	w(w),
	h(h),
	type(physicsim::Rectangle) {
}

physicsim::RigidBody::RigidBody(float x, float y, float m, float theta, float r)
	: pos(2, 1, { x, y }),
	m(m),
	theta(theta),
	lVel(2, 1),
	aVel(0),
	w(w),
	h(h),
	type(physicsim::Circle) {
}
physicsim::Shape physicsim::RigidBody::getType() const {
	return this->type;
}

void physicsim::RigidBody::addImpulse(physicsim::Matrix i) {
	this->lVel = this->lVel + i.scalarDivide(this->m); //divide impulse by mass
}


float physicsim::RigidBody::getH() const {
	if (this->type == physicsim::Circle) {
		return -1;
	}
	return this->h;
}

float physicsim::RigidBody::getW() const {
	if (this->type == physicsim::Circle) {
		return -1;
	}
	return this->w;
}

float physicsim::RigidBody::getR() const {
	if (this->type == physicsim::Rectangle) {
		return -1;
	}
	return this->r;
}

float physicsim::RigidBody::getT() const {
	return this->theta;
}

physicsim::Matrix(*physicsim::RigidBody::getVertices()) [4] {
	return &this->vertices;
}

physicsim::Matrix physicsim::RigidBody::getPos() const {
	return this->pos;
}
