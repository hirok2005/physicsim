#include "physicsim/collisions.hpp"
#include "physicsim/rigidbody.hpp"
#include "physicsim/constants.hpp"

// todo broad phase detection
/*! Returns 1 when objects overlap the same coordinates in space. Currently works with only 2 bodies
 *
 * TODO: collision detection for multiple shapes, and homogenous interface
 */
bool physicsim::Collisions::collisionDetect(RigidBody* body1, RigidBody* body2){
	if (body1->getType() == physicsim::Circle && body2->getType() == physicsim::Circle) {
		return physicsim::Collisions::circleCircleCollisionDetect(body1, body2);
	}
	if (body1->getType() == physicsim::Rectangle && body2->getType() == physicsim::Rectangle) {
		return physicsim::Collisions::rectRectCollisionDetect(body1, body2);
	}
	if (body1->getType() == physicsim::Rectangle && body2->getType() == physicsim::Circle) {
		return physicsim::Collisions::rectCircleCollisionDetect(body1, body2);
	}
	if (body1->getType() == physicsim::Circle&& body2->getType() == physicsim::Rectangle) {
		return physicsim::Collisions::rectCircleCollisionDetect(body2, body1);
	}
	return 0;
}

// todo param validation on these
/*! Collision detection for circles
 */
bool physicsim::Collisions::circleCircleCollisionDetect(RigidBody* circle1, RigidBody* circle2) {
	return std::pow(circle1->getR() + circle2->getR(), 2) > (circle1->getPos() - circle2->getPos()).vectorMagnitudeSqrd();
}

/*! Collision detection for circle + rectangle
	this isnt that optimal i think
	shouls come back and optimise later
	issue: when close but not intersecting it says it is
 */
bool physicsim::Collisions::rectCircleCollisionDetect(RigidBody* rect, RigidBody* circle) {
	physicsim::Matrix localCirclePos = physicsim::rotationMat2D(2 * physicsim::PI - rect->getT()) * (circle->getPos() - rect->getPos());
	localCirclePos(0, 0) = std::abs(localCirclePos(0, 0));
	localCirclePos(1, 0) = std::abs(localCirclePos(1, 0));

	if (localCirclePos(0, 0) > (rect->getW() / 2 + circle->getR())) {
		return false;
	}
	if (localCirclePos(1, 0) > (rect->getH() / 2 + circle->getR())) {
		return false;
	}

	if (localCirclePos(0, 0) <= (rect->getW() / 2)) {
		return true;
	}
	if (localCirclePos(1, 0) <= (rect->getH() / 2)) {
		return true;
	}

	float cornerDist = std::pow(localCirclePos(0, 0) - rect->getW() / 2, 2) +
		std::pow(localCirclePos(1, 0) - rect->getH() / 2, 2);

	return (cornerDist <= std::pow(circle->getR(), 2));
}


/*! Collision detection for rectangles
 */
bool physicsim::Collisions::rectRectCollisionDetect(RigidBody* rect1, RigidBody* rect2) {
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

/*! Iterate over all pairs of bodies and alter motion between each pair of bodies
 *
 * TODO: implement getMomentum() for rigidbody
 */
void physicsim::Collisions::collisionHandler(float dt) {

}