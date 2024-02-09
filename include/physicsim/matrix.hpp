// todo:
//		make int type consistent, lazy

#ifndef PHYSICSIM_MATRIX_HPP
#define PHYSICSIM_MATRIX_HPP

#include "physicsim.hpp"

namespace physicsim {
	class Matrix {
		public:
			int rows; int cols;
			float* arr; // matrix is stored as 1 contiguous array
			Matrix(int rows, int cols);
			void insert(int row, int col, float val);
			float retrieve(int row, int col) const;
			Matrix transpose();
			Matrix operator+(const Matrix& other) const;
			Matrix operator-(const Matrix& other) const;
			Matrix operator*(const Matrix& other) const;
	};
}

#endif // PHYSICSIM_MATRIX_HPP
