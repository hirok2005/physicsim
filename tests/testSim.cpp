#include "physicsim/physicsim.hpp"
#include <chrono>
#include <cmath>
#include <iostream>

int main() {
	// setup
	physicsim::World sim(150, 100);
	std::cout << "0" << std::endl;
	sim.addBody(new physicsim::RigidBody(50, 90, 1, 0.5, 10, 10));
	std::cout << "1" << std::endl;
	sim.addBody(new physicsim::RigidBody(20, 90, 5, 0, 10, 10));
	std::cout << "2" << std::endl;
	// sim.addBody(new physicsim::RigidBody(5, 5, 4, 0, 1, 1));
	sim.bodies[1]->setLVel({ 2, 1, {10, 0} });
	sim.bodies[0]->setLVel({ 2, 1, {5, 0} });
	physicsim::Renderer ren(&sim, true);
	double dt = 0;
	while (ren.window.isOpen()) {
		if (sim.bodies[0]->getPos()(0, 0) > sim.X) {
			sim.bodies[0]->setLVel({ 2, 1, {-10, 0} });
		}
		else if (sim.bodies[0]->getPos()(0, 0) < 0) {
			sim.bodies[0]->setLVel({ 2, 1, {10, 0} });
		}
		if (sim.bodies[1]->getPos()(0, 0) > sim.X) {
			sim.bodies[1]->setLVel({ 2, 1, {-5, 0} });
		}
		else if (sim.bodies[1]->getPos()(0, 0) < 0) {
			sim.bodies[1]->setLVel({ 2, 1, {5, 0} });
		}
		auto startTime = std::chrono::high_resolution_clock::now();
		physicsim::Matrix pos = sim.bodies[0]->getPos();
		sim.step(dt);

		sf::Event event;
		while (ren.window.pollEvent(event)) {
			if (event.type == sf::Event::Closed) //for more events, switch case for event
				ren.window.close();
		}
		ren.update(dt);
		auto endTime = std::chrono::high_resolution_clock::now();
		dt = std::chrono::duration_cast<std::chrono::duration<double>>(endTime - startTime).count();
		//sim.collisionHandler(dt);
		std::cout << sim.collisionDetect(sim.bodies[0], sim.bodies[1]) << "\n";
	}

	return 0;
}
