#include "physicsim/rigidbody.hpp"
#include "physicsim/constants.hpp"
#include "physicsim/matrix.hpp"
#include <stdexcept>

// todo: store radius square and inverse mass

/*! Constructs a rigidobody with given position, mass, angle from horizontal and shape
 */
physicsim::RigidBody::RigidBody(double x, double y, double m, double theta, physicsim::Shape type, double aVel, double torque, double e) {
	this->pos = physicsim::Matrix(2, 1, {x, y}); //set x and y values
	this->m = m;
	this->theta = theta;
	this->lVel = physicsim::Matrix(2, 1, {0, 0});
	this->aVel = aVel;
	this->torque = torque;
	this->type = type;
	this->invM = 1 / m;
	this->e = e;
}

/*! Constructs a rectangle rigidbody with given position, mass, angle from horizontal and dimensions
 */
physicsim::RigidBody::RigidBody(double x, double y, double m, double theta, double w, double h, double aVel, double torque, double e) : vertices{physicsim::Matrix(2, 1, {-w / 2, h / 2}), physicsim::Matrix(2, 1, {w / 2, h / 2}), physicsim::Matrix(2, 1, {w / 2, -h / 2}), physicsim::Matrix(2, 1, {-w / 2, -h / 2})} {
	this->pos = physicsim::Matrix(2, 1, { x, y }); //set x and y values
	this->m = m;
	this->theta = theta;
	this->lVel = physicsim::Matrix(2, 1, { 0, 0 });
	this->f = physicsim::Matrix(2, 1, { 0, 0 });
	this->aVel = aVel;
	this->torque = torque;
	this->type = type;
	this->w = w;
	this->h = h;
	this->type = physicsim::Rectangle;
	this->invM = 1 / m;
	this->I = 12 / (m * (h * h + w * w)); // inverse moment of inertia
	this->torque = 0;
	this->e = e;
}

/*! Construct a circle rigidbody with position, mass, angle and radius
*/
physicsim::RigidBody::RigidBody(double x, double y, double m, double theta, double r, double aVel, double torque, double e) {
	this->pos = physicsim::Matrix(2, 1, { x, y }); //set x and y values
	this->m = m;
	this->theta = theta;
	this->lVel = physicsim::Matrix(2, 1, { 0, 0 });
	this->f = physicsim::Matrix(2, 1, { 0, 0 });
	this->aVel = aVel;
	this->torque = torque;
	this->type = type;
	this->r = r;
	this->type = physicsim::Circle;
	this->invM = 1 / m;
	this->I = 2 / (m * r * r);
	this->e = e;
}

physicsim::Shape physicsim::RigidBody::getType() const {
	return this->type;
}

/*! Changes velocity based on given impulse
 */
void physicsim::RigidBody::addImpulse(physicsim::Matrix i, double dt) {
	this->f = i.scalarDivide(dt); //Impulse to force. Update will take care of applying the force. Not as efficient as directly turning impulse to velocity
}

void physicsim::RigidBody::addAVel(const double &aVel) {
	this->aVel += aVel;
}

void physicsim::RigidBody::addTorque(const double &torque) {
	this->torque += torque;
}

physicsim::Matrix physicsim::RigidBody::getMomentum() {
	return this->lVel.scalarMultiply(this->m);
}

/*! Applies uniform acceleration for each timestep, updates position and resets force
 */
void physicsim::RigidBody::update(double dt) {
	this->lVel += this->f.scalarDivide(this->m).scalarMultiply(dt);
	this->pos += this->lVel.scalarMultiply(dt);
	this->f = physicsim::Matrix(2, 1, { 0, 0 });
	this->theta += this->aVel * dt;
	this->aVel += this->torque * this->I;
	this->torque = 0;
}

double physicsim::RigidBody::getH() const {
	if (this->type == physicsim::Circle) {
		return -1;
	}
	return this->h;
}

double physicsim::RigidBody::getW() const {
	if (this->type == physicsim::Circle) {
		return -1;
	}
	return this->w;
}

double physicsim::RigidBody::getR() const {
	if (this->type == physicsim::Rectangle) {
		return -1;
	}
	return this->r;
}

double physicsim::RigidBody::getT() const {
	return this->theta;
}

double physicsim::RigidBody::getI() const {
    return this->I;
}

double physicsim::RigidBody::getAVel() const {
    return this->aVel;
}

double physicsim::RigidBody::getTorque() const {
    return this->torque;
}

double physicsim::RigidBody::getInvM() const {
    return this->invM;
}

physicsim::Matrix physicsim::RigidBody::getLVel() const {
    return this->lVel;
}

double physicsim::RigidBody::getTheta() const {
	return this->theta;
}

double physicsim::RigidBody::getE() const {
return this->e;
}

void physicsim::RigidBody::setLVel(const Matrix &lVel) {
    this->lVel = lVel;
}

void physicsim::RigidBody::setF(const Matrix& f) {
	this->f = f;
}

void physicsim::RigidBody::setPos(const Matrix& pos) {
	this->pos = pos;
}

void physicsim::RigidBody::setTheta(const double& theta) {
	this->theta = theta;
}

void physicsim::RigidBody::setAVel(const double &aVel) {
	this->aVel = aVel;
}

void physicsim::RigidBody::setTorque(const double &torque) {
	this->torque = torque;
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

void physicsim::RigidBody::addTheta(const double& theta) {
	this->theta += theta;
	// todo: bound to |theta| <= 2pi 
}

physicsim::Matrix(*physicsim::RigidBody::getVertices()) [4] {
	return &this->vertices;
}

void physicsim::RigidBody::getVerticesWorld(physicsim::Matrix(&res)[4]) {
	physicsim::Matrix rotMat = physicsim::rotationMat2D(this->theta);

	for (int i = 0; i < 4; i++) {
		res[i] = (rotMat * this->vertices[i]) + this->pos;
	}
}

physicsim::Matrix physicsim::RigidBody::getPos() const {
	return this->pos;
}
