// todo:
//		choose a good scale
//		adding/removing objects to renderer after construction

#include "physicsim/render.hpp"
#include "physicsim/world.hpp"
#include "physicsim/constants.hpp"
#include <string>
#include <numbers>
#include <stdlib.h>

/*! Converts position of body in physicsim::vector to sf::Vector2f
 *  Uses physicsim::SCALE as scaling factor
 */
sf::Vector2f physicsim::Renderer::sfPosition(const physicsim::RigidBody& body) const {
	Matrix pos = body.getPos();
	return sf::Vector2f(pos(0, 0) * physicsim::SCALE, this->height - pos(1, 0) * physicsim::SCALE);
}

/*! Constructor for renderer
 *
 * Takes a world, which is our virtual representation of a simulation manager
 * Convert all world dimensions to the dimensions for rendering
 * Flag showInfo for diagnostics
 */
physicsim::Renderer::Renderer(World* sim, bool showInfo) {
	std::srand(time(NULL));
	this->showInfo= showInfo;
	this->sim = sim;
	this->width = sim->X * physicsim::SCALE; this->height = sim->Y * physicsim::SCALE;
	this->window.create(sf::VideoMode(this->width, this->height), "Renderer", sf::Style::Default);

	// not sure about paths, this doenst work right now and its late, maybe require user to put in PATH vars, also maybe average it so its somewhat visible
	if (this->font.loadFromFile("/home/hirok/dev/physicsim/arial.ttf")) {
		this->info.setFont(this->font);
		this->info.setCharacterSize(20);
		this->info.setPosition(0, this->height - 20);
		this->info.setFillColor(sf::Color::White);
	}
	for (physicsim::RigidBody* body : sim->bodies) {
		if (body->getType() == physicsim::Rectangle) {
			this->shapes.push_back(new physicsim::ShapeGroup({ new sf::RectangleShape(sf::Vector2f(body->getW() * physicsim::SCALE, body->getH() * physicsim::SCALE)) }));
		}
		else {
			this->shapes.push_back(new physicsim::ShapeGroup({ new sf::CircleShape(body->getR() * physicsim::SCALE) }));
		}
		this->shapes.back()->shapes.back()->setFillColor(sf::Color(std::rand() % 256, std::rand() % 256, std::rand() % 256));
		if (body->getType() == physicsim::Rectangle) {
			this->shapes.back()->setOrigin(sf::Vector2((float)(physicsim::SCALE * body->getW() / 2), (float)(physicsim::SCALE * body->getH() / 2)));
		}
		else {
			// ugly and not centered, fix later for aesthetics
			this->shapes.back()->addShape(new sf::CircleShape(10, 3));
			this->shapes.back()->shapes.back()->setPosition(sf::Vector2f(physicsim::SCALE * body->getR(), 0));
			this->shapes.back()->shapes.back()->setFillColor(sf::Color::Blue);
			this->shapes.back()->setOrigin(sf::Vector2f(physicsim::SCALE * body->getR(), physicsim::SCALE * body->getR()));
			
		}
	}

}

/*! Redraws the shapes on the screen with new scaled position
 *  Called every render cycle
 *  Can show time delta from processing
 */
void physicsim::Renderer::update(const double& dt) {
	this->window.clear();
	for (int i = 0; i < this->shapes.size(); i++) {
		this->shapes[i]->setPosition(this->sfPosition(*this->sim->bodies[i]));
		this->shapes[i]->setRotation(this->sim->bodies[i]->getT() * 180 / std::numbers::pi);
		this->window.draw(*this->shapes[i]);
	}
	if (this->showInfo) {
		this->info.setString(std::to_string(1 / dt));
		this->window.draw(this->info);
	}
	this->window.display();
}



physicsim::ShapeGroup::ShapeGroup(std::initializer_list<sf::Shape*> shapes) {
	for (int i = 0; i < shapes.size(); i++) {
		this->shapes.push_back(*(shapes.begin() + i)); //pointer/iterator magic
	}
}
void physicsim::ShapeGroup::addShape(sf::Shape* shape) {
	this->shapes.push_back(shape);
	// this->shapes.back()->setOrigin(this->shapes[0]->getOrigin());
}

void physicsim::ShapeGroup::draw(sf::RenderTarget& target, sf::RenderStates states) const {
	for (int i = 0; i < this->shapes.size(); i++) {
		target.draw(*this->shapes[i], states);
	}
}

void physicsim::ShapeGroup::setPosition(sf::Vector2f position) {
	for (int i = 0; i < this->shapes.size(); i++) {
		this->shapes[i]->setPosition(position);
	}
}

void physicsim::ShapeGroup::setOrigin(sf::Vector2f position) {
	this->shapes[0]->setOrigin(position);
}

void physicsim::ShapeGroup::setRotation(double angle) {
	for (int i = 0; i < this->shapes.size(); i++) {
		this->shapes[i]->setRotation(angle);
	}
}

