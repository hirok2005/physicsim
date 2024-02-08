#ifndef PHYSICSIM_VECTOR_HPP
#define PHYSICSIM_VECTOR_HPP

#include "physicsim.hpp"

namespace physicsim {
	class Vector {
		public:
			float x, y;
			Vector(float x, float y);
			float distanceSq() const;
			float dot(const Vector& other) const;
			Vector operator+(const Vector& other) const;
			Vector operator-(const Vector& other) const;
	};
}

#endif // PHYSICSIM_VECTOR_HPP
