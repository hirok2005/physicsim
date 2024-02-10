#include "physicsim/render.hpp"

namespace physicsim {
	void runRenderLoop() {
        sf::RenderWindow window(
            sf::VideoMode(640, 480),
            "Hello World");
        sf::CircleShape shape(200);
        sf::RectangleShape shape2(sf::Vector2f(100, 100));
        while (window.isOpen()) {
            sf::Event event;
            while (window.pollEvent(event)) {
                if (event.type == sf::Event::Closed) {
                    window.close();
                }
            }

            window.clear();
            window.draw(shape);
            window.draw(shape2);
            window.display();
        }
	}
}
