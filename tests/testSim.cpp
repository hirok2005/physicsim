#include "physicsim/physicsim.hpp"
#include <chrono>
#include <iostream>

int main() {
	// setup
	physicsim::World sim(100, 100);
	sim.addBody(new physicsim::RigidBody(50, 100, 1, 0, 2, 2));
	// sim.addBody(new physicsim::RigidBody(5, 5, 4, 0, 1, 1));

	sim.bodies[0]->addF({ 2, 1, {0, -2 * 9.81} });
	physicsim::Renderer ren(&sim);
	double dt = 0;

	while (ren.window.isOpen()) {
		auto startTime = std::chrono::high_resolution_clock::now();
		physicsim::Matrix pos = sim.bodies[0]->getPos();
		std::cout << pos(1, 0) << "\n";
		sim.step(dt);

		sf::Event event;
		while (ren.window.pollEvent(event)) {
			if (event.type == sf::Event::Closed) //for more events, switch case for event
				ren.window.close();
		}
		ren.update();

		auto endTime = std::chrono::high_resolution_clock::now();
		dt = std::chrono::duration_cast<std::chrono::duration<double>>(endTime - startTime).count();
	
	}

	return 0;
}
