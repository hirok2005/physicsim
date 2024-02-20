#ifndef PHYSICSIM_RENDER_HPP
#define PHYSICSIM_RENDER_HPP

#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include "world.hpp"



// 1 pixel = 0.5 metres?
namespace physicsim {
	class Renderer {
		public:
			World* sim;
			int width, height;

			Renderer(World* sim);
			void update();
		private:
			sf::RenderWindow window;
	};

}

#endif // PHYSICSIM_RENDER_HPP
