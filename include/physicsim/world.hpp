#ifndef PHYSICSIM_WORLD_HPP
#define PHYSICSIM_WORLD_HPP

#include "rigidbody.hpp"
#include <vector>

namespace physicsim {
	class World {
		public:
			const int X, Y; // world dimensions
			std::vector<RigidBody*> bodies;

			World(int x, int y);
			void addBody(RigidBody* body);
			void step(float dt); // run at each loop

			bool collisionDetect(RigidBody* body1, RigidBody* body2) const;
			void collisionHandler(); // everything collision related in this, simply call this every loop and is handles it all

	};
}

#endif // PHYSICSIM_WORLD_HPP
