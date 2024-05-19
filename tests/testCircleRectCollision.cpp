#include "physicsim/physicsim.hpp"
#include <chrono>
#include <cmath>
#include <numbers>
#include <iostream>

int main() {
    // setup
    physicsim::World sim(150, 100);
    physicsim::Matrix coords[4];
    sim.addBody(new physicsim::RigidBody(10, 10, 1, 0.5, 5, 0, 0, 0));
    sim.addBody(new physicsim::RigidBody(50, 50, 0, 0, 30, 30, 0, 0, 0));
    sim.addBody(new physicsim::RigidBody(10, 10, 1, 0, 1, 0, 0, 0));
    physicsim::Renderer ren(&sim, true);
    double dt = 0;
    while (ren.window.isOpen()) {
        auto startTime = std::chrono::high_resolution_clock::now();
        physicsim::Matrix pos = sim.bodies[0]->getPos();

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
        physicsim::Collisions::Manifold man;
        std::cout << physicsim::Collisions::rectCircleCollisionDetect(sim.bodies[1], sim.bodies[0], man) << "\n";
        // sim.bodies[1]->addTheta(0.0001);
        // sim.bodies[2]->setPos(man.point);
    }

    return 0;
}