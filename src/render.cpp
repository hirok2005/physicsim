// todo:
//		choose a good scale
//		adding/removing objects to renderer after construction

#include "physicsim/render.hpp"
#include "physicsim/world.hpp"
#include <iostream>
#include <string>

physicsim::Renderer::Renderer(World* sim, bool showInfo) {
	this->showInfo= showInfo;
	this->sim = sim;
	this->width = sim->X * physicsim::SCALE; this->height = sim->Y * physicsim::SCALE;
	this->window.create(sf::VideoMode(this->width, this->height), "SFML Window", sf::Style::Default, sf::ContextSettings(0, 0, 8));

	// not sure about paths, this doenst work right now and its late, also maybe average it so its somewhat visible
	if (!this->font.loadFromFile("C:/dev/cpp_dev/physicsim/arial.ttf")) {
		std::cout << "cant find file";
	} else {
		this->info.setFont(this->font);
		this->info.setCharacterSize(20);
		this->info.setPosition(0, this->height - 20);
		this->info.setFillColor(sf::Color::White);
	}
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

void physicsim::Renderer::update(const float& dt) {
	this->window.clear();
	physicsim::Matrix pos;
	for (int i = 0; i < this->shapes.size(); i++) {
		pos = this->sim->bodies[i]->getPos();
		this->shapes[i]->setPosition(pos(0, 0) * physicsim::SCALE, this->height - pos(1, 0) * physicsim::SCALE);
		this->shapes[i]->setRotation(this->sim->bodies[i]->getT());
		this->window.draw(*this->shapes[i]);
	}
	if (this->showInfo) {
		this->info.setString(std::to_string(1 / dt));
		this->window.draw(this->info);
	}
	this->window.display();
}
