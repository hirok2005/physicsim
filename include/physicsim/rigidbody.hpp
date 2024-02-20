#ifndef PHYSICSIM_RIGIDBODY_HPP
#define PHYSICSIM_RIGIDBODY_HPP

#include "matrix.hpp"

//TODO:	alter position based on velocity and timedelta
//	time variables for calculating delta

namespace physicsim {
	enum Shape {Circle, Rectangle, Default};

	class RigidBody {
		private:
			float m, theta, aVel;
			Matrix pos, lVel;
			Shape type;
		public:
			RigidBody(float x, float y, float m, float theta, Shape type);
			~RigidBody() = default;
			Shape getType();
			void addImpulse(Matrix i);
	};

	class RigidRect : public RigidBody {
		private:
			float w, h;
			Matrix vertices[4]; // local coords
		public:
			RigidRect(float x, float y, float m, float theta, float w, float h);
	};

	class RigidCircle : public RigidBody {
		private:
			float r;
		public:
			RigidCircle(float x, float y, float m, float theta, float r);
	};

}

#endif // PHYSICSIM_RIGIDBODY_HPP
