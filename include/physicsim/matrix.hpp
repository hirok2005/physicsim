// todo:
//		make int type consistent, lazy
//		sort into private, public

#ifndef PHYSICSIM_MATRIX_HPP
#define PHYSICSIM_MATRIX_HPP

namespace physicsim {
	class Matrix {
		public:
			int rows; int cols;
			float* arr; // matrix is stored as 1 contiguous array
			Matrix(); //default constructor, sets rows and cols to zero
			Matrix(int rows, int cols);
			Matrix(int rows, int cols, float* arr); //second constructor for when you have all values, consider std::initialiser_list
			void insert(int row, int col, float val);
			float retrieve(int row, int col) const;
			void swapRows(int row1, int row2);
			Matrix transpose();

			Matrix scalarMultiply(float lambda) const;
			Matrix scalarDivide(float lambda) const; //shorthand for easier usage, essentially just matrix.scalarMultiply(1/lambda)

			Matrix operator+(const Matrix& other) const; //maybe some +=, +=, etc operators would be nice
			Matrix operator-(const Matrix& other) const;
			Matrix operator*(const Matrix& other) const;

			int boundsCheck(int row, int col); //private method
	};
}

#endif // PHYSICSIM_MATRIX_HPP
