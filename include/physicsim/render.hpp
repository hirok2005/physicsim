#ifndef PHYSICSIM_RENDER_HPP
#define PHYSICSIM_RENDER_HPP

#include <vector>
#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include "world.hpp"



// 1 pixel = 0.01 cm?
namespace physicsim {

	constexpr int SCALE = 10;

	class Renderer {
		public:
			World* sim;
			int width, height;
			std::vector<sf::Shape*> shapes;

			Renderer(World* sim);
			void update();
			sf::RenderWindow window;
	};

}

#endif // PHYSICSIM_RENDER_HPP
