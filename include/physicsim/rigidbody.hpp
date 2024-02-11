#ifndef PHYSICSIM_RIGIDBODY_HPP
#define PHYSICSIM_RIGIDBODY_HPP

#include "matrix.hpp"

namespace physicsim {
	class RigidBody {
		public:
			float m, theta;
			Matrix pos;
			RigidBody(float x, float y, float m, float theta);
	};
}

#endif // PHYSICSIM_RIGIDBODY_HPP
