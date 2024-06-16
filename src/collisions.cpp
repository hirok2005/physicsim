#include "physicsim/collisions.hpp"
#include "physicsim/matrix.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <vector>
/*
*  Structure to store information for a given collision, allows a simple interface for resolving functions
*/

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
		// return false;
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
	physicsim::Matrix edge, edgeNorm, c1, c2, c3, r1, r2, pointOnEdge, closestPoint;
	double dist, dotProd, t;
	double minDist = std::numeric_limits<double>::infinity();
	rect->getVerticesWorld(verts);
	for (int i = 0; i < 2; i++) {
		edge = verts[i + 1] - verts[i];
		edgeNorm = edge.normalise(); // so we dont need to keep normalising for projection

		// only really need to project the current vertices
		// for n vertex polygons / none regular we would need to go over all
		r1 = verts[i].project(edgeNorm, true);
		r2 = verts[i + 1].project(edgeNorm, true);
		c1 = circle->getPos() + edgeNorm.scalarMultiply(circle->getR());
		c2 = circle->getPos() - edgeNorm.scalarMultiply(circle->getR());
		if (!physicsim::Collisions::sat(edgeNorm, { r1, r2 }, { c1, c2 }, t, manifold)) {
			return false;
		}
		t = edge.vectorMagnitudeSqrd();
		dotProd = std::min(std::max(edge.dot(circle->getPos() - verts[i]) / t, 0.0), 1.0);
		pointOnEdge = verts[i] + edge.scalarMultiply(dotProd);
		dist = (circle->getPos() - pointOnEdge).vectorMagnitudeSqrd();
		if (dist < minDist) {
			minDist = dist;
			closestPoint = pointOnEdge;
		}
	}

	// can optimize this, todo
	for (int i = 2; i < 4; i++) {
		edge = verts[(i + 1) % 4] - verts[i];
		t = edge.vectorMagnitudeSqrd();
		dotProd = std::min(std::max(edge.dot(circle->getPos() - verts[i]) / t, 0.0), 1.0);
		pointOnEdge = verts[i] + edge.scalarMultiply(dotProd);
		dist = (circle->getPos() - pointOnEdge).vectorMagnitudeSqrd();
		if (dist < minDist) {
			minDist = dist;
			closestPoint = pointOnEdge;
		}
	}
	edge = closestPoint - circle->getPos();
	bool pass = false;
	if (edge(0, 0) == 0 && edge(1, 0) == 0) {
		pass = true;
	}
	edge = edge.normalise();
	// if (edge.dot(rect->getLVel() - circle->getLVel()) > 0) {
	// 	return false;
	// }
	if (!pass && !physicsim::Collisions::sat(edge, { verts[0] , verts[1], verts[2], verts[3] }, { circle->getPos() + edge.scalarMultiply(circle->getR()), circle->getPos() - edge.scalarMultiply(circle->getR()) }, t, manifold)) {
		return false;
	}


	manifold.normal = edge;
	manifold.depth = circle->getR() - std::sqrt(minDist);
	return true;
}



/*! Collision detection for rectangles
 */
bool physicsim::Collisions::rectRectCollisionDetect(physicsim::RigidBody* rect1, physicsim::RigidBody* rect2, physicsim::Collisions::Manifold& manifold) {
	physicsim::Matrix verts1[4];
	rect1->getVerticesWorld(verts1);
	physicsim::Matrix verts2[4];
	rect2->getVerticesWorld(verts2);
	double depth;
	double minDepth = std::numeric_limits<double>::infinity();
	physicsim::Matrix norm;
	physicsim::Matrix edge;
	int ind;
	for (int i = 0; i < 2; i++) {
		edge = verts1[(i + 1) % 4] - verts1[i];
		edge = edge.normalise();
		if (!physicsim::Collisions::sat(edge, { verts1[i], verts1[(i + 1) % 4] }, { verts2[0], verts2[1], verts2[2], verts2[3] }, depth, manifold)) {
			return false;
		}
		if (depth < minDepth) {
			minDepth = depth;
			norm = edge;
			ind = manifold.index;
		}
		edge = verts2[(i + 1) % 4] - verts2[i];
		edge = edge.normalise();
		if (!physicsim::Collisions::sat(edge, { verts2[i], verts2[(i + 1) % 4] }, { verts1[0], verts1[1], verts1[2], verts1[3] }, depth, manifold)) {
			return false;
		}
		if (depth < minDepth) {
			minDepth = depth;
			norm = edge;
			ind = -manifold.index;
		}
	}
	manifold.depth = minDepth;
	manifold.normal = norm;
	manifold.index = ind;
	if (manifold.normal.dot(rect2->getPos() - rect1->getPos()) < 0) {
		manifold.normal = manifold.normal.scalarMultiply(-1);
	}
	return true;
}

/*
 * call after normal and depth have been found
 * and bodies have been pushed
*/
void physicsim::Collisions::findContactPoints(physicsim::RigidBody* body1, physicsim::RigidBody* body2, physicsim::Collisions::Manifold& manifold) {
	if (body1->getType() == physicsim::Circle && body2->getType() == physicsim::Circle) {
		return physicsim::Collisions::circleCircleContact(body1, body2, manifold);
	}
	if (body1->getType() == physicsim::Rectangle && body2->getType() == physicsim::Rectangle) {
		return physicsim::Collisions::rectRectContact(body1, body2, manifold);
	}
	if (body1->getType() == physicsim::Rectangle && body2->getType() == physicsim::Circle) {
		return physicsim::Collisions::rectCircleContact(body1, body2, manifold);
	}
	if (body1->getType() == physicsim::Circle && body2->getType() == physicsim::Rectangle) {
		return physicsim::Collisions::rectCircleContact(body2, body1, manifold);
	}
}

void physicsim::Collisions::circleCircleContact(physicsim::RigidBody* body1, physicsim::RigidBody* body2, physicsim::Collisions::Manifold& manifold) {
	manifold.point = body1->getPos() + manifold.normal.scalarMultiply(body1->getR());
}

void physicsim::Collisions::rectRectContact(physicsim::RigidBody* body1, physicsim::RigidBody* body2, physicsim::Collisions::Manifold& manifold) {
	physicsim::Matrix points[4];
	if (manifold.index > 0) {
		body2->getVerticesWorld(points);
		manifold.point = points[manifold.index - 1];
		return;
	}
	body1->getVerticesWorld(points);
	manifold.point = points[std::abs(manifold.index + 1)];
}

void physicsim::Collisions::rectCircleContact(physicsim::RigidBody* rect, physicsim::RigidBody* circle, physicsim::Collisions::Manifold& manifold) {
	manifold.point = circle->getPos() + manifold.normal.scalarMultiply(circle->getR());
}

void physicsim::Collisions::positionalCorrection(physicsim::RigidBody* body1, physicsim::RigidBody* body2, const physicsim::Collisions::Manifold& manifold) {
	physicsim::Matrix correction = manifold.normal.scalarMultiply((0.5 * std::abs(manifold.depth)) / (body1->getInvM() + body2->getInvM()));
	body1->addPos(correction.scalarMultiply(-body1->getInvM()));
	body2->addPos(correction.scalarMultiply(body2->getInvM()));
}

/* Takes in 2 bodies, checks if there is a collision, if there is one, it is resolved
 *
 */
void physicsim::Collisions::collisionHandler(RigidBody* body1, RigidBody* body2, double dt) {
	physicsim::Collisions::Manifold manifold;
	if (body2->getType() == physicsim::Circle) {
		physicsim::RigidBody* t = body2;
		body2 = body1;
		body1 = t;
	}
	if (!physicsim::Collisions::collisionDetect(body1, body2, manifold)) {
		return;
	}

	// push bodies apart create manifold and resolve

	physicsim::Collisions::positionalCorrection(body1, body2, manifold);
	physicsim::Collisions::findContactPoints(body1, body2, manifold);
	physicsim::Collisions::resolve(body1, body2, manifold);
}



// https://research.ncl.ac.uk/game/mastersdegree/gametechnologies/previousinformation/physics6collisionresponse/2017%20Tutorial%206%20-%20Collision%20Response.pdf
void physicsim::Collisions::resolve(RigidBody* body1, RigidBody* body2, const Manifold& manifold) {
	// Calculate relative velocity along the collision normal
	double relativeVelocity = (body2->getLVel() - body1->getLVel()).dot(manifold.normal);

	// Coefficient of restitution
	double e = std::min(body1->getE(), body2->getE());

	// Initial impulse calculation
	double J = -(1 + e) * relativeVelocity;

	// Calculate the divisor considering both linear and angular effects
	double divisor = manifold.normal.vectorMagnitudeSqrd() * (body1->getInvM() + body2->getInvM());

	// Perpendicular distances from the collision point to the bodies' centers of mass
	physicsim::Matrix body1ToColPerp = (manifold.point - body1->getPos()).perp();
	physicsim::Matrix body2ToColPerp = (manifold.point - body2->getPos()).perp();

	// Add rotational contribution to the divisor
	double r1PerpDotN = body1ToColPerp.dot(manifold.normal);
	double r2PerpDotN = body2ToColPerp.dot(manifold.normal);

	divisor += body1->getI() * (r1PerpDotN * r1PerpDotN) + body2->getI() * (r2PerpDotN * r2PerpDotN);

	// Finalize impulse magnitude
	J /= divisor;

	// Apply impulse to linear velocities
	body1->addLVel(manifold.normal.scalarMultiply(-J * body1->getInvM()));
	body2->addLVel(manifold.normal.scalarMultiply(J * body2->getInvM()));

	// Apply impulse to angular velocities
	if (body1->getI() > 0) {
		body1->addAVel(r1PerpDotN * J * body1->getI());
	}
	if (body2->getI() > 0) {
		body2->addAVel(r2PerpDotN * -J * body2->getI());
	}
}

bool physicsim::Collisions::sat(const physicsim::Matrix& edge, std::vector<physicsim::Matrix> shape1, std::vector<physicsim::Matrix> shape2, double& depth, physicsim::Collisions::Manifold& manifold) {
	for (int i = 0; i < shape1.size(); i++) {
		shape1[i] = shape1[i].project(edge, true);
	}
	for (int i = 0; i < shape2.size(); i++) {
		shape2[i] = shape2[i].project(edge, true);
	}
	double min1, max1, min2, max2;

	int minInd, maxInd;
	physicsim::Collisions::minMax(edge, shape1, min1, max1, minInd, maxInd);
	physicsim::Collisions::minMax(edge, shape2, min2, max2, minInd, maxInd);

	depth = max1 - min2;
	double overlap2 = max2 - min1;
	manifold.index = minInd + 1;
	if (overlap2 < depth) {
		depth = overlap2;
		manifold.index = maxInd+ 1;
	}
	// depth = std::min(max1, max2)- std::min(min1, min2);

	return !(max1 < min2 || max2 < min1);
}

/*
  takes in a vector of projected points and the normal of the subspace
  retuns the min and max vector
  no of points must be >= 2
  assumings vectors are 2D
*/
void physicsim::Collisions::minMax(const physicsim::Matrix& axis, const std::vector<physicsim::Matrix>& points, double& min, double& max, int& minInd, int& maxInd) {
	min = std::numeric_limits<double>::infinity();
	max = -std::numeric_limits<double>::infinity();
	int i = 0;
	for (const physicsim::Matrix& point : points) {
		// Assuming the points are already projections onto the axis
		double projection = point.dot(axis);
		if (projection < min) {
			min = projection;
			minInd = i;
		}
		if (projection > max) {
			max = projection;
			maxInd = i;
		}
		i++;
	}
}
