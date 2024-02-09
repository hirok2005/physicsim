#ifndef PHYSICSIM_WORLD_HPP
#define PHYSICSIM_WORLD_HPP

#include "physicsim.hpp"

namespace physicsim {
	class World {
		public:
			void step(int dt); // run at each loop
	};
}

#endif // PHYSICSIM_WORLD_HPP
