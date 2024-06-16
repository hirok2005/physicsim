#include "physicsim/physicsim.hpp"
#include <chrono>
#include <cmath>
#include <iostream>

int main() {
	// setup
	physicsim::World sim(725, 86);
	sim.addBody(new physicsim::RigidBody(40, 50, 5, 0, 30, 30, 0, 0, 0.5));
	sim.addBody(new physicsim::RigidBody(80, 50, 5, 0, 30, 30, 0, 0, 0.5));
	// sim.addBody(new physicsim::RigidBody(10, 10, 2, 0.5, 10, 0, 0, 0.5));
	// sim.addBody(new physicsim::RigidBody(0, 20, 1, 0, 1, 0, 0, 0.5));
	// sim.addBody(new physicsim::RigidBody(80, 10, 2, 0.5, 5, 0, 0, 0.5));
	// sim.addBody(new physicsim::RigidBody(100, 10, 1, 0, 1, 0, 0, 0.5));
	// sim.addBody(new physicsim::RigidBody(117, 10, 5, 0, 5, 5, 0, 0, 0.5));
	// sim.addBody(new physicsim::RigidBody(117, 70, 5, 0, 5, 5, 0, 0, 0.5));
	// sim.addBody(new physicsim::RigidBody(85, 65, 5, 0, 5, 5, 0, 0, 0.5));
	physicsim::Renderer ren(&sim, true);
	double dt = 0;
	while (ren.window.isOpen()) {
		auto startTime = std::chrono::high_resolution_clock::now();
		sim.step(dt);

		sf::Event event;
		while (ren.window.pollEvent(event)) {
			if (event.type == sf::Event::Closed) { //for more events, switch case for event
				ren.window.close();
			}
			// Handle key presses
			if (event.type == sf::Event::KeyPressed) {
				if (event.key.code == sf::Keyboard::Left) {
					sim.bodies[0]->addLVel(physicsim::Matrix(2, 1, { -2, 0 })); // Apply force to the left
				}
				else if (event.key.code == sf::Keyboard::Right) {
					sim.bodies[0]->addLVel(physicsim::Matrix(2, 1, { 2, 0 })); // Apply force to the right
				}
				else if (event.key.code == sf::Keyboard::Up) {
					sim.bodies[0]->addLVel(physicsim::Matrix(2, 1, { 0, 2 })); // Apply force upwards
				}
				else if (event.key.code == sf::Keyboard::Down) {
					sim.bodies[0]->addLVel(physicsim::Matrix(2, 1, { 0, -2 })); // Apply force downwards
				}
				else if (event.key.code == sf::Keyboard::A) {
					sim.bodies[0]->addAVel(0.1); // Apply force to the left
				}
				else if (event.key.code == sf::Keyboard::D) {
					sim.bodies[0]->addAVel(-0.1); // Apply force to the right
				}
				else if (event.key.code == sf::Keyboard::Space) {
					sim.bodies[0]->addLVel(sim.bodies[0]->getLVel().scalarMultiply(-1));
				}
			}
		}

		ren.update(dt);
		auto endTime = std::chrono::high_resolution_clock::now();
		dt = std::chrono::duration_cast<std::chrono::duration<double>>(endTime - startTime).count();
	}

	return 0;
}
