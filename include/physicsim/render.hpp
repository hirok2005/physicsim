#ifndef PHYSICSIM_RENDER_HPP
#define PHYSICSIM_RENDER_HPP

#include <vector>
#include <deque>
#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include <SFML/Graphics/Drawable.hpp>
#include "world.hpp"



namespace physicsim {
	constexpr int SCALE = 10;
	class ShapeGroup : public sf::Drawable {
		private:
		public:
			std::vector<sf::Shape*> shapes;
			ShapeGroup(std::initializer_list<sf::Shape*> shapes);
			void addShape(sf::Shape* shape);
			virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const;
			virtual void setPosition(sf::Vector2f position);
			virtual void setOrigin(sf::Vector2f position);
			virtual void setRotation(double angle);
	};


	class Renderer {
		public:
			World* sim;
			int width, height;
			std::vector<physicsim::ShapeGroup*> shapes;
			bool showInfo;
			// Buffer to store recent FPS values
			std::deque<double> fpsBuffer;
			const size_t fpsBufferSize = 100; // Number of frames to average over

			double getAverageFPS() const;

			sf::Vector2f sfPosition(const RigidBody& body) const;
			Renderer(World* sim, bool showFPS);
			void update(const double& dt);
			sf::RenderWindow window;
			sf::Text info;
			sf::Font font;
	};
}
#endif // PHYSICSIM_RENDER_HPP
