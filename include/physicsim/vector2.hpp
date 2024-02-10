#ifndef PHYSICSIM_VECTOR2_HPP
#define PHYSICSIM_VECTOR2_HPP

namespace physicsim {
	class Vector2 {
		public:
			float x, y;
			Vector2(float x, float y);
			Vector2(const Vector2& other);
			float distance() const;
			float dot(const Vector2& other) const;
			void scalarMult(const float scalar);
			void norm();
			Vector2 operator+(const Vector2& other) const;
			Vector2 operator-(const Vector2& other) const;
	};
}

#endif // PHYSICSIM_VECTOR2_HPP
