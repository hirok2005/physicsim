#include "physicsim/physicsim.hpp"
#include <chrono>
#include <cmath>
#include <iostream>

int main() {
	// setup
	physicsim::World sim(150, 100);
	sim.addBody(new physicsim::RigidBody(10, 10, 1, 0, 5));
	sim.addBody(new physicsim::RigidBody(50, 50, 1, 0, 30, 30));
	physicsim::Renderer ren(&sim, true);
	double dt = 0;
	while (ren.window.isOpen()) {
		auto startTime = std::chrono::high_resolution_clock::now();
		physicsim::Matrix pos = sim.bodies[0]->getPos();
		sim.step(dt);

		sf::Event event;
		while (ren.window.pollEvent(event)) {
			if (event.type == sf::Event::Closed) { //for more events, switch case for event
				ren.window.close();
			}
			// Handle key presses
			if (event.type == sf::Event::KeyPressed) {
				if (event.key.code == sf::Keyboard::Left)
					sim.bodies[0]->addPos(physicsim::Matrix(2, 1, { -0.5, 0 })); // Apply force to the left
				else if (event.key.code == sf::Keyboard::Right)
					sim.bodies[0]->addPos(physicsim::Matrix(2, 1, { 0.5, 0 })); // Apply force to the right
				else if (event.key.code == sf::Keyboard::Up)
					sim.bodies[0]->addPos(physicsim::Matrix(2, 1, { 0, 0.5 })); // Apply force upwards
				else if (event.key.code == sf::Keyboard::Down)
					sim.bodies[0]->addPos(physicsim::Matrix(2, 1, { 0, -0.5 })); // Apply force downwards
			}
		}

		ren.update(dt);
		auto endTime = std::chrono::high_resolution_clock::now();
		dt = std::chrono::duration_cast<std::chrono::duration<double>>(endTime - startTime).count();
		//sim.collisionHandler(dt);
		std::cout << physicsim::Collisions::collisionDetect(sim.bodies[0], sim.bodies[1]) << "\n";
	}

	return 0;
}
