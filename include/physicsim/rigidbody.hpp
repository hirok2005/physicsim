#ifndef PHYSICSIM_RIGIDBODY_HPP
#define PHYSICSIM_RIGIDBODY_HPP

#include "matrix.hpp"

//TODO:	alter position based on velocity and timedelta
//	time variables for calculating delta

namespace physicsim {
	enum Shape {Circle, Rectangle, Default};

	class RigidBody {
		protected:
			float m, theta, aVel, torque, invM, I, e; // mass, angle of rotation, angular velocity, torque, moment of inertia, coeff of elasticity
			Matrix pos, lVel, f; // global position, linear velocities, forces
			Shape type;

			// for circles
			float r = -1;

			// for rects
			float w = -1, h = -1;
			Matrix vertices[4]; // local coords
		
		public:
			RigidBody(float x, float y, float m, float theta, Shape type, float aVel, float torque, float e);
			RigidBody(float x, float y, float m, float theta, float w, float h, float aVel, float torque, float e);
			RigidBody(float x, float y, float m, float theta, float r, float aVel, float torque, float e);
			~RigidBody() = default;
			Shape getType() const;
			float getW() const;
			float getH() const;
			float getR() const;
			float getT() const;
			float getI() const;
			float getAVel() const;
			float getTorque() const;
			float getInvM() const;
			Matrix getLVel() const;
			float getE() const;
			void setLVel(const Matrix& lVel);
			void setF(const Matrix& f);
			void setPos(const Matrix& pos);
			void setTheta(const float& theta);
			void setAVel(const float& aVel);
			void setTorque(const float& torque);
			void addLVel(const Matrix& lVel);
			void addF(const Matrix& f);
			void addPos(const Matrix& pos);
			void addTheta(const float& theta);
			void addImpulse(Matrix i, float dt);
			void addAVel(const float& aVel);
			void addTorque(const float& torque);
			void update(float dt);

			Matrix getMomentum();
			Matrix(*getVertices()) [4];
			void getVerticesWorld(physicsim::Matrix(&res)[4]); // in world coords
			Matrix getPos() const;
	};
}

#endif // PHYSICSIM_RIGIDBODY_HPP
