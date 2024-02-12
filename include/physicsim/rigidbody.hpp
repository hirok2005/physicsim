#ifndef PHYSICSIM_RIGIDBODY_HPP
#define PHYSICSIM_RIGIDBODY_HPP

#include "matrix.hpp"

//TODO:	alter position based on velocity and timedelta
//	time variables for calculating delta

namespace physicsim {
	class RigidBody {
		private:
			float m, theta;
			Matrix pos, vel;
		public:
			RigidBody(float x, float y, float m, float theta);
			~RigidBody() = default;

			void addImpulse(Matrix i);
	};
}

#endif // PHYSICSIM_RIGIDBODY_HPP
