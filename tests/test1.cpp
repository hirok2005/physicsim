#include <physicsim/physicsim.hpp>
#include <iostream>


int main() {
	physicsim::Matrix t(3, 3);

	physicsim::Matrix a(2, 2, {0,0,0,0});
	a(1,1) = 5;

	std::cout << a.retrieve(1,1) << std::endl;

	try {
		physicsim::Matrix b(2, 2, {1,1,1,1,1});
	}
	catch (std::exception e) {
		std::cout << "error" << std::endl;
	}

	int x = 0;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			t.insert(i, j, ++x);
		}
	}

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			std::cout << t.retrieve(i, j) << ", ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;

	t = t.transpose();

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			std::cout << t.retrieve(i, j) << ", ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;


	// ChatGPT generated
	physicsim::Matrix m1(2, 3);
	m1.insert(0, 0, 1); m1.insert(0, 1, 2); m1.insert(0, 2, 3);
	m1.insert(1, 0, 4); m1.insert(1, 1, 5); m1.insert(1, 2, 6);

	physicsim::Matrix m2(3, 2);
	m2.insert(0, 0, 7); m2.insert(0, 1, 8);
	m2.insert(1, 0, 9); m2.insert(1, 1, 10);
	m2.insert(2, 0, 11); m2.insert(2, 1, 12);

	// Perform multiplication
	physicsim::Matrix result = m1 * m2;
	// expected:
	// 58, 64
	// 139, 154

	for (int i = 0; i < 2; ++i) {
		for (int j = 0; j < 2; ++j) {
			std::cout << result.retrieve(i, j) << ", ";
		}
		std::cout << std::endl;
	}

	return 0;

}
