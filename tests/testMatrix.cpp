#include <physicsim/physicsim.hpp>
#include <iostream>
#include <cmath>


void printMatrix(physicsim::Matrix mat) {
	for (int i = 0; i < mat.getRows(); i++) {
		for (int j = 0; j < mat.getCols(); j++) {
			std::cout << mat(j, i) << ", ";
		}
		std::cout << "\n";
	}
	std::cout << "\n";
}


int main() {
	physicsim::Matrix t(3, 3);

	physicsim::Matrix a(2, 2, { 0,0,0,0 });
	a(1, 1) = 5;

	std::cout << a.retrieve(1, 1) << std::endl;

	printMatrix(a);

	try {
		physicsim::Matrix b(2, 2, { 1,1,1,1,1 });
	}
	catch (std::exception e) {
		std::cout << e.what() << std::endl;
	}

	int x = 0;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			t.insert(i, j, ++x);
		}
	}

	printMatrix(t);

	t = t.transpose();
	
	printMatrix(t);

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

	printMatrix(result);

	physicsim::Matrix rot = physicsim::Matrix::rotationMat2D(3.14159 / 2);

	printMatrix(rot);

	physicsim::Matrix b(3, 3, { 1, 2, 3, 4, 5, 6, 7, 8, 9 });

	printMatrix(b);

	return 0;

}
