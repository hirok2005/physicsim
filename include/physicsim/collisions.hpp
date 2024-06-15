#ifndef PHYSICSIM_COLLISIONS_HPP
#define PHYSICSIM_COLLISIONS_HPP

#include "rigidbody.hpp"
#include "matrix.hpp"
#include <vector>

namespace physicsim {
	namespace Collisions {
		struct Manifold {
			double depth;
			Matrix normal;
			Matrix point;
		};

		bool collisionDetect(RigidBody* body1, RigidBody* body2, Manifold& manifold);
		void collisionHandler(RigidBody* body1, RigidBody* body2, double dt); // everything collision related in this, simply call this every loop and is handles it all
		bool circleCircleCollisionDetect(RigidBody* circle1, RigidBody* circle2, Manifold& manifold);
		bool rectCircleCollisionDetect(RigidBody* rect, RigidBody* circle, Manifold& manifold);
		bool rectRectCollisionDetect(RigidBody* rect1, RigidBody* rect2, Manifold& manifold);
		void findContactPoints(RigidBody* body1, RigidBody* body2, Manifold& manifold);
		void circleCircleContact(RigidBody* body1, RigidBody* body2, Manifold& manifold);
		void rectRectContact(RigidBody* body1, RigidBody* body2, Collisions::Manifold& manifold);
		void rectCircleContact(RigidBody* rect, RigidBody* circle, Collisions::Manifold& manifold);
		void positionalCorrection(RigidBody* body1, RigidBody* body2, const Manifold& manifold);
		void collisionHandler(RigidBody* body1, RigidBody* body2, double dt);
		void resolve(RigidBody* body1, RigidBody* body2, const Manifold& manifold);
		bool sat(const Matrix& edge, std::vector<Matrix> shape1, std::vector<Matrix> shape2, double& depth);
		void minMax(const Matrix& axis, const std::vector<Matrix>& points, double& min, double& max);
	}
};

#endif // !PHYSICSIM_COLLISIONS_HPP
