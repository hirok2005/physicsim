// todo:
//		choose a good scale
//		adding/removing objects to renderer after construction

#include "physicsim/render.hpp"
#include "physicsim/world.hpp"
#include <iostream>

physicsim::Renderer::Renderer(World* sim) {
	this->sim = sim;
	this->width = sim->X * physicsim::SCALE; this->height = sim->Y * physicsim::SCALE;
	this->window.create(sf::VideoMode(this->width, this->height), "SFML Window", sf::Style::Default, sf::ContextSettings(0, 0, 8));

	for (physicsim::RigidBody* body : sim->bodies) {
		if (body->getType() == physicsim::Rectangle) {
			this->shapes.push_back(new sf::RectangleShape(sf::Vector2f(body->getW() * physicsim::SCALE, body->getH() * physicsim::SCALE)));
		}
		else {
			this->shapes.push_back(new sf::CircleShape(body->getR() * physicsim::SCALE));
		}
		this->shapes.back()->setFillColor(sf::Color::White);
	}

}

void physicsim::Renderer::update() {
	window.clear();
	physicsim::Matrix pos;
	for (int i = 0; i < this->shapes.size(); i++) {
		pos = sim->bodies[i]->getPos();
		this->shapes[i]->setPosition(pos(0, 0) * physicsim::SCALE, pos(1, 0) * physicsim::SCALE);
		this->shapes[i]->setRotation(sim->bodies[i]->getT());
		window.draw(*this->shapes[i]);
	}
	
	window.display();
}
