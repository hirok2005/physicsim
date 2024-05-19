#include "physicsim/collisions.hpp"
#include "physicsim/constants.hpp"
#include "physicsim/matrix.hpp"
#include <cmath>
#include <limits>
#include <iostream>
/*
*  Structure to store information for a given collision, allows a simple interface for resolving functions
*/
struct physicsim::Collisions::Manifold;

// todo broad phase detection
/*! Returns 1 when objects overlap the same coordinates in space. Currently works with only 2 bodies
 *
 * TODO: collision detection for multiple shapes, and homogenous interface
 */
bool physicsim::Collisions::collisionDetect(physicsim::RigidBody* body1, physicsim::RigidBody* body2, physicsim::Collisions::Manifold& manifold) {
	if (body1->getType() == physicsim::Circle && body2->getType() == physicsim::Circle) {
		return physicsim::Collisions::circleCircleCollisionDetect(body1, body2, manifold);
	}
	if (body1->getType() == physicsim::Rectangle && body2->getType() == physicsim::Rectangle) {
		return physicsim::Collisions::rectRectCollisionDetect(body1, body2, manifold);
	}
	if (body1->getType() == physicsim::Rectangle && body2->getType() == physicsim::Circle) {
		return physicsim::Collisions::rectCircleCollisionDetect(body1, body2, manifold);
	}
	if (body1->getType() == physicsim::Circle && body2->getType() == physicsim::Rectangle) {
		return physicsim::Collisions::rectCircleCollisionDetect(body2, body1, manifold);
	}
	return 0;
}


// todo param validation on these
/*! Collision detection for circles
 */
bool physicsim::Collisions::circleCircleCollisionDetect(physicsim::RigidBody* circle1, physicsim::RigidBody* circle2, physicsim::Collisions::Manifold& manifold) {
	physicsim::Matrix norm = circle2->getPos() - circle1->getPos();

	double dist = norm.vectorMagnitudeSqrd(); // manually do it to not do redundant stuff

	if (std::pow(circle1->getR() + circle2->getR(), 2) < dist) {
		return false;
	}

	dist = std::sqrt(dist);
	norm = norm.scalarMultiply(1 / dist);


	if (norm.dot(circle2->getLVel() - circle1->getLVel()) > 0) {
		return false;
	}

	double depth = dist - (circle1->getR() + circle2->getR());
	// physicsim::Matrix point = body1->getPos() + norm.scalarMultiply(body1->getR());

	manifold.depth = depth;
	manifold.normal = norm;
	return true;
}

/*! switched to SAT
*/
bool physicsim::Collisions::rectCircleCollisionDetect(physicsim::RigidBody* rect, physicsim::RigidBody* circle, physicsim::Collisions::Manifold& manifold) {
	physicsim::Matrix verts[4];
	physicsim::Matrix edge, c1, c2, r1, r2;
	double dist;
	rect->getVerticesWorld(verts);
	for (int i = 0; i < 4; i++) {
		edge = verts[(i + 1) % 4] - verts[i];
		edge = edge.normalise(); // so we dont need to keep normalising for projection

		// only really need to project the current vertices
		// for n vertex polygons / none regular we would need to go over all
		r1 = verts[i].project(edge, true);
		r2 = verts[(i + 1) % 4].project(edge, true);
		c1 = (circle->getPos() + edge.scalarMultiply(circle->getR())).project(edge, true);
		c2 = (circle->getPos() - edge.scalarMultiply(circle->getR())).project(edge, true);
		dist = (r2 - r1).vectorMagnitudeSqrd();
		// std::cout << i << ", " << dist << ", " << (c1 - r1).vectorMagnitudeSqrd() << ", " << (c1 - r2).vectorMagnitudeSqrd() << ", " << (c2 - r1).vectorMagnitudeSqrd() << ", " << (c2 - r2).vectorMagnitudeSqrd() << "\n";
		
		if (((c1 - r1).vectorMagnitudeSqrd() > dist || (c1 - r2).vectorMagnitudeSqrd() > dist) && ((c2 - r1).vectorMagnitudeSqrd() > dist || (c2 - r2).vectorMagnitudeSqrd() > dist)) {
			return false;
		}
		// if distance to r1 < dist and distance to r2 < dist then in

	}

	return true;
}



/*! Collision detection for rectangles
 */
bool physicsim::Collisions::rectRectCollisionDetect(physicsim::RigidBody* rect1, physicsim::RigidBody* rect2, physicsim::Collisions::Manifold& manifold) {
	// SAT using body1s edges first
	// get points, convert to Collisions coords, find normal of edge
	// not sure if we have to do it for both rects
	physicsim::Matrix CollisionsVertsB1[4];
	rect1->getVerticesWorld(CollisionsVertsB1);
	physicsim::Matrix CollisionsVertsB2[4];
	rect2->getVerticesWorld(CollisionsVertsB2);

	physicsim::Matrix edge;
	physicsim::Matrix norm;
	for (int i = 0; i < 4; i++) {
		bool passed = true;
		edge = CollisionsVertsB1[(i + 1) % 4] - CollisionsVertsB1[i];
		norm = { 2, 1, {-1 * edge(1, 0), edge(0, 0)} };
		for (int j = 0; j < 4; j++) {
			if ((CollisionsVertsB2[j] - CollisionsVertsB1[i]).dot(norm) < 0) {
				passed = false;
			}
		}
		if (passed) {
			return false;
		}
	}

	return true;
}

void physicsim::Collisions::positionalCorrection(physicsim::RigidBody* body1, physicsim::RigidBody* body2, const physicsim::Collisions::Manifold& manifold) {
	physicsim::Matrix correction = manifold.normal.scalarMultiply((0.5 * manifold.depth) / (body1->getInvM() + body2->getInvM()));
	body1->addPos(correction.scalarMultiply(body1->getInvM()));
	body2->addPos(correction.scalarMultiply(-body2->getInvM()));
}

/* Takes in 2 bodies, checks if there is a collision, if there is one, it is resolved
 *
 * TODO: implement getMomentum() for rigidbody
 */
void physicsim::Collisions::collisionHandler(RigidBody* body1, RigidBody* body2, double dt) {
	physicsim::Collisions::Manifold manifold;
	if (!physicsim::Collisions::collisionDetect(body1, body2, manifold)) {
		return;
	}

	// push bodies apart create manifold and resolve

	physicsim::Collisions::positionalCorrection(body1, body2, manifold);
	// find contact point
	physicsim::Collisions::resolve(body1, body2, manifold);
}



// https://research.ncl.ac.uk/game/mastersdegree/gametechnologies/previousinformation/physics6collisionresponse/2017%20Tutorial%206%20-%20Collision%20Response.pdf
void physicsim::Collisions::resolve(RigidBody* body1, RigidBody* body2, const Manifold& manifold) {
	double J = -(1 + std::min(body1->getE(), body2->getE())) * (body2->getLVel() - body1->getLVel()).dot(manifold.normal);
	double divisor = manifold.normal.vectorMagnitudeSqrd() * (body1->getInvM() + body2->getInvM());
	physicsim::Matrix body1ToColPerp = (manifold.point - body1->getPos()).perp();
	physicsim::Matrix body2ToColPerp = (manifold.point - body2->getPos()).perp();

	// pretty jank right now, when I implement the other types will be more concise
  // note circle circle collisions dont need rotation stuff, but now we can use this for the other collisions
	if (body1->getType() == physicsim::Circle && body1->getType() == physicsim::Circle) {
		divisor += body1->getI() * std::pow(body2ToColPerp.dot(manifold.normal), 2) + body2->getI() * std::pow(body2ToColPerp.dot(manifold.normal), 2);
		J /= divisor;

		body1->addLVel(manifold.normal.scalarMultiply(-J * body1->getInvM()));
		body2->addLVel(manifold.normal.scalarMultiply(J * body2->getInvM()));
		body1->addAVel(body1ToColPerp.dot(manifold.normal) * J * body1->getI());
		body2->addAVel(body2ToColPerp.dot(manifold.normal) * J * body2->getI());
	}
}
