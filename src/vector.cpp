#include "physicsim/vector.hpp"


physicsim::Vector::Vector(float x, float y) {
	this->x = x;
	this->y = y;
}

float physicsim::Vector::distanceSq() const {
	return (this->x * this->x + this->y * this->y);
}

float physicsim::Vector::dot(const physicsim::Vector& other) const {
	return (this->x * other.x + this->y + other.y);
}

physicsim::Vector physicsim::Vector::operator+(const physicsim::Vector& other) const {
	return physicsim::Vector(this->x + other.x, this->y + other.y);
}

physicsim::Vector physicsim::Vector::operator-(const physicsim::Vector& other) const {
	return physicsim::Vector(this->x - other.x, this->y - other.y);
}



