#ifndef PHYSICSIM_RIGIDBODY_HPP
#define PHYSICSIM_RIGIDBODY_HPP

#include "matrix.hpp"

//TODO:	alter position based on velocity and timedelta
//	time variables for calculating delta

namespace physicsim {
	enum Shape {Circle, Rectangle, Default};

	class RigidBody {
		protected:
			double m, theta, aVel, torque, invM, I, e; // mass, angle of rotation, angular velocity, torque, moment of inertia, coeff of elasticity
			Matrix pos, lVel, f; // global position, linear velocities, forces
			Shape type;

			// for circles
			double r = -1;

			// for rects
			double w = -1, h = -1;
			Matrix vertices[4]; // local coords
		
		public:
			RigidBody(double x, double y, double m, double theta, Shape type, double aVel, double torque, double e);
			RigidBody(double x, double y, double m, double theta, double w, double h, double aVel, double torque, double e);
			RigidBody(double x, double y, double m, double theta, double r, double aVel, double torque, double e);
			~RigidBody() = default;
			Shape getType() const;
			double getW() const;
			double getH() const;
			double getR() const;
			double getT() const;
			double getI() const;
			double getAVel() const;
			double getTorque() const;
			double getInvM() const;
			Matrix getLVel() const;
			double getE() const;
			void setLVel(const Matrix& lVel);
			void setF(const Matrix& f);
			void setPos(const Matrix& pos);
			void setTheta(const double& theta);
			void setAVel(const double& aVel);
			void setTorque(const double& torque);
			void addLVel(const Matrix& lVel);
			void addF(const Matrix& f);
			void addPos(const Matrix& pos);
			void addTheta(const double& theta);
			void addImpulse(Matrix i, double dt);
			void addAVel(const double& aVel);
			void addTorque(const double& torque);
			void update(double dt);

			Matrix getMomentum();
			Matrix(*getVertices()) [4];
			void getVerticesWorld(physicsim::Matrix(&res)[4]); // in world coords
			Matrix getPos() const;
	};
}

#endif // PHYSICSIM_RIGIDBODY_HPP
