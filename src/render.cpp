// todo:
//		choose a good scale
//		adding/removing objects to renderer after construction

#include "physicsim/render.hpp"
#include "physicsim/world.hpp"
#include "physicsim/constants.hpp"
#include <string>

sf::Vector2f physicsim::Renderer::sfPosition(const RigidBody& body) const {
	Matrix pos = body.getPos();
	if (body.getType() == physicsim::Rectangle) {
		return sf::Vector2f((pos(0, 0)) * physicsim::SCALE, this->height - pos(1, 0) * physicsim::SCALE);
	}
	else {
		return sf::Vector2f(pos(0, 0) * physicsim::SCALE, this->height - pos(1, 0) * physicsim::SCALE);
	}
}

physicsim::Renderer::Renderer(World* sim, bool showInfo) {
	this->showInfo= showInfo;
	this->sim = sim;
	this->width = sim->X * physicsim::SCALE; this->height = sim->Y * physicsim::SCALE;
	this->window.create(sf::VideoMode(this->width, this->height), "Renderer", sf::Style::Default);

	// not sure about paths, this doenst work right now and its late, maybe require user to put in PATH vars, also maybe average it so its somewhat visible
	if (this->font.loadFromFile("C:/dev/cpp_dev/physicsim/arial.ttf")) {
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
		if (body->getType() == physicsim::Rectangle) {
			this->shapes.back()->setOrigin(physicsim::SCALE * body->getW() / 2, physicsim::SCALE * body->getH() / 2);
		}
		else {
			this->shapes.back()->setOrigin(physicsim::SCALE * body->getR(), physicsim::SCALE * body->getR());
		}
	}

}

void physicsim::Renderer::update(const float& dt) {
	this->window.clear();
	for (int i = 0; i < this->shapes.size(); i++) {
		this->shapes[i]->setPosition(this->sfPosition(*this->sim->bodies[i]));
		this->shapes[i]->setRotation(this->sim->bodies[i]->getT() * physicsim::RAD_TO_DEG);
		this->window.draw(*this->shapes[i]);
	}
	if (this->showInfo) {
		this->info.setString(std::to_string(1 / dt));
		this->window.draw(this->info);
	}
	this->window.display();
}
