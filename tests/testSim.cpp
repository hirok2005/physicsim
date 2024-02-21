#include "physicsim/physicsim.hpp"

int main() {
	physicsim::World sim(10, 10);
	sim.addBody(new physicsim::RigidBody(1, 1, 2, 0, 2, 2));
	physicsim::Renderer ren(&sim);


	// should be ready for physics stuff now, just need to have sim.update or whatever now
	while (ren.window.isOpen()) {
		sf::Event event;
		while (ren.window.pollEvent(event)) {
			if (event.type == sf::Event::Closed)
				ren.window.close();
		}
		ren.update();
	}

	return 0;
}
