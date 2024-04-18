#include "physicsim/physicsim.hpp"
#include <chrono>
#include <cmath>
#include <iostream>

int main() {
	// setup
	physicsim::World sim(150, 100);
	sim.addBody(new physicsim::RigidBody(10, 10, 100, 0, 5, 0, 0, 1));
	sim.addBody(new physicsim::RigidBody(50, 10, 100, 0, 5, 0, 0, 0.5));
	sim.addBody(new physicsim::RigidBody(10, 30, 100, 0, 5, 0, 0, 2));
	sim.addBody(new physicsim::RigidBody(100, 30, 100, 0, 5, 0, 0, 2));
	sim.addBody(new physicsim::RigidBody(30, 30, 100, 0, 5, 0, 0, 0.01));
	sim.addBody(new physicsim::RigidBody(60.5, 10, 1000, 0, 5, 0, 0, 1));
	sim.bodies[0]->addLVel(physicsim::Matrix(2, 1, { 10, 0 }));
	sim.bodies[2]->addLVel(physicsim::Matrix(2, 1, { 10, 0 }));
	sim.bodies[3]->addLVel(physicsim::Matrix(2, 1, { -10, 0 }));
	physicsim::RigidBody* t = new physicsim::RigidBody(50, 10, 100, 0, 5, 0, 0, 1);
	sim.addBody(t);
	sim.addBody(new physicsim::RigidBody(50, 50, 100, 0, 5, 0, 0, 1));
	sim.addBody(new physicsim::RigidBody(46.5, 57.5, 100, 0, 5, 0, 0, 1));
	sim.addBody(new physicsim::RigidBody(53.5, 57.6, 100, 0, 5, 0, 0, 1));
	// sim.addBody(new physicsim::RigidBody(10, 10, 0.0000000000001, 0, 5, 0, 0, .5));
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
