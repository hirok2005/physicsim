#include "physicsim/vector2.hpp"


physicsim::Vector2::Vector2(float x, float y) {
	this->x = x;
	this->y = y;
}

physicsim::Vector2::Vector2(const physicsim::Vector2& other) {
	this->x = other.x;
	this->y = other.y;
}

float physicsim::Vector2::distance() const {
	return (this->x * this->x + this->y * this->y);
}

float physicsim::Vector2::dot(const physicsim::Vector2& other) const {
	return (this->x * other.x + this->y + other.y);
}

void physicsim::Vector2::scalarMult(const float scalar) {
	this->x *= scalar; this->y *= scalar;
}

void physicsim::Vector2::norm() {
	float d = this->distance();
	this->scalarMult(1 / d);
}

physicsim::Vector2 physicsim::Vector2::operator+(const physicsim::Vector2& other) const {
	return physicsim::Vector2(this->x + other.x, this->y + other.y);
}

physicsim::Vector2 physicsim::Vector2::operator-(const physicsim::Vector2& other) const {
	return physicsim::Vector2(this->x - other.x, this->y - other.y);
}



