#ifndef PHYSICSIM_COLLISIONS_HPP
#define PHYSICSIM_COLLISIONS_HPP

#include "rigidbody.hpp"
#include "matrix.hpp"

namespace physicsim {
	namespace Collisions {
		struct Manifold {
			float depth;
			Matrix normal;
			Matrix point;
		};

		bool collisionDetect(RigidBody* body1, RigidBody* body2);
		void collisionHandler(RigidBody* body1, RigidBody* body2, float dt); // everything collision related in this, simply call this every loop and is handles it all
		bool circleCircleCollisionDetect(RigidBody* circle1, RigidBody* circle2);
		bool rectCircleCollisionDetect(RigidBody* rect, RigidBody* circle);
		bool rectRectCollisionDetect(RigidBody* rect1, RigidBody* rect2);
		void push(RigidBody* body1, RigidBody* body2, const float depth, const Matrix& normal);
		void push(RigidBody* body1, RigidBody* body2, const float depth) {
			void collisionHandler(RigidBody * body1, RigidBody * body2, float dt);
			void getContactPoint(Manifold * collision, RigidBody * body1, RigidBody * body2);
		}

	}
}

#endif // !PHYSICSIM_COLLISIONS_HPP