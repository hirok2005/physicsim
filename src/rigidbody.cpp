#include "physicsim/rigidbody.hpp"
#include "physicsim/matrix.hpp"
#include <stdexcept>


physicsim::RigidBody::RigidBody(float x, float y, float m, float theta, physicsim::Shape type) {
	this->pos = physicsim::Matrix(2, 1, {x, y}); //set x and y values
	this->m = m;
	this->theta = theta;
	this->lVel = physicsim::Matrix(2, 1, {0, 0});
	this->aVel = 0;
	this->type = type;
}

physicsim::RigidBody::RigidBody(float x, float y, float m, float theta, float w, float h) : vertices{ physicsim::Matrix(2, 1, {-w / 2, h / 2}), physicsim::Matrix(2, 1, { w / 2, h / 2 }), physicsim::Matrix(2, 1, { w / 2, -h / 2 }), physicsim::Matrix(2, 1, { -w / 2, -h / 2 }) } {
	this->pos = physicsim::Matrix(2, 1, { x, y }); //set x and y values
	this->m = m;
	this->theta = theta;
	this->lVel = physicsim::Matrix(2, 1, { 0, 0 });
	this->f = physicsim::Matrix(2, 1, { 0, 0 });
	this->aVel = 0;
	this->type = type;
	this->w = w;
	this->h = h;
	this->type = physicsim::Rectangle;
}

physicsim::RigidBody::RigidBody(float x, float y, float m, float theta, float r) {
	this->pos = physicsim::Matrix(2, 1, { x, y }); //set x and y values
	this->m = m;
	this->theta = theta;
	this->lVel = physicsim::Matrix(2, 1, { 0, 0 });
	this->f = physicsim::Matrix(2, 1, { 0, 0 });
	this->aVel = 0;
	this->type = type;
	this->r = r;
	this->type = physicsim::Circle;

}
physicsim::Shape physicsim::RigidBody::getType() const {
	return this->type;
}

void physicsim::RigidBody::addImpulse(physicsim::Matrix i) {
	this->lVel = this->lVel + i.scalarDivide(this->m); //divide impulse by mass
}

void physicsim::RigidBody::update(float dt) {
	this->lVel += this->f.scalarDivide(this->m).scalarMultiply(dt);
	this->pos += this->lVel.scalarMultiply(dt);
	this->f = physicsim::Matrix(2, 1, { 0, 0 });
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

void physicsim::RigidBody::setLVel(const Matrix& lVel) {
	this->lVel = lVel;
}

void physicsim::RigidBody::setF(const Matrix& f) {
	this->f = f;
}

void physicsim::RigidBody::setPos(const Matrix& pos) {
	this->pos = pos;
}

void physicsim::RigidBody::setTheta(const float& theta) {
	this->theta = theta;
}

void physicsim::RigidBody::addLVel(const Matrix& lVel) {
	this->lVel += lVel;
}

void physicsim::RigidBody::addF(const Matrix& f) {
	this->f += f;
}

void physicsim::RigidBody::addPos(const Matrix& pos) {
	this->pos += pos;
}

void physicsim::RigidBody::addTheta(const float& theta) {
	this->theta += theta;
}

physicsim::Matrix(*physicsim::RigidBody::getVertices()) [4] {
	return &this->vertices;
}

physicsim::Matrix physicsim::RigidBody::getPos() const {
	return this->pos;
}
