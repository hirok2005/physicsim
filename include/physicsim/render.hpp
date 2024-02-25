#ifndef PHYSICSIM_RENDER_HPP
#define PHYSICSIM_RENDER_HPP

#include <vector>
#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include "world.hpp"



namespace physicsim {

	constexpr int SCALE = 10;

	class Renderer {
		public:
			World* sim;
			int width, height;
			std::vector<sf::Shape*> shapes;
			bool showInfo;

			Renderer(World* sim, bool showFPS);
			void update(const float& dt);
			sf::RenderWindow window;
			sf::Text info;
			sf::Font font;
	};

}

#endif // PHYSICSIM_RENDER_HPP
