#ifndef PHYSICSIM_RIGIDBODY_HPP
#define PHYSICSIM_RIGIDBODY_HPP

#include "matrix.hpp"

//TODO:	alter position based on velocity and timedelta
//	time variables for calculating delta

namespace physicsim {
	enum Shape {Circle, Rectangle, Default};

	class RigidBody {
		protected:
			float m, theta, aVel;
			Matrix pos, lVel;
			Shape type;

			// for circles
			float r = NULL;

			// for rects
			float w = NULL, h = NULL;
			Matrix vertices[4]; // local coords
		public:
			RigidBody(float x, float y, float m, float theta, Shape type);
			RigidBody(float x, float y, float m, float theta, float w, float h);
			RigidBody(float x, float y, float m, float theta, float r);
			~RigidBody() = default;
			Shape getType() const;
			float getW() const;
			float getH() const;
			float getR() const;
			float getT() const;
			void addImpulse(Matrix i);
			Matrix(*getVertices()) [4];
			Matrix getPos() const;
	};
}

#endif // PHYSICSIM_RIGIDBODY_HPP
