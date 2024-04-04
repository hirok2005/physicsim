#ifndef PHYSICSIM_COLLISIONS_HPP
#define PHYSICSIM_COLLISIONS_HPP

#include "rigidbody.hpp"

namespace physicsim {
	namespace Collisions {
		bool collisionDetect(RigidBody* body1, RigidBody* body2);
		void collisionHandler(float dt); // everything collision related in this, simply call this every loop and is handles it all
		bool circleCircleCollisionDetect(RigidBody* circle1, RigidBody* circle2);
		bool rectCircleCollisionDetect(RigidBody* rect, RigidBody* circle);
		bool rectRectCollisionDetect(RigidBody* rect1, RigidBody* rect2);
	}
	
}

#endif // !PHYSICSIM_COLLISIONS_HPP


// functions for all collision things
