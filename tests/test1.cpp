#include <physicsim/physicsim.hpp>
#include <iostream>


int main() {
	physicsim::runRenderLoop();
	physicsim::Vector v(1, 2);
	physicsim::Vector u(1, 2);

	float d = v.dot(u);
	u + v;

	std::cout << v.x << ", " << v.y << ", " << d;

	return 0;

}
