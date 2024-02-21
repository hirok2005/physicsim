// todo:
//		make int type consistent, lazy
//		sort into private, public

#ifndef PHYSICSIM_MATRIX_HPP
#define PHYSICSIM_MATRIX_HPP

#include <initializer_list>
#include <cmath>

namespace physicsim {
	class Matrix {
		private:
			int rows; int cols;
			float* arr; // matrix is stored as 1 contiguous array
		public:
			Matrix(); //default constructor, sets rows and cols to zero
			Matrix(int rows, int cols);
			Matrix(int rows, int cols, std::initializer_list<float> vals); //second constructor for when you have all values, consider std::initialiser_list
			void insert(int row, int col, float val); //deprecated with []
			
			float retrieve(int row, int col) const; //deprecated with []
			void swapRows(int row1, int row2);
			Matrix transpose();

			Matrix scalarMultiply(float lambda) const;
			Matrix scalarDivide(float lambda) const; //shorthand for easier usage, essentially just matrix.scalarMultiply(1/lambda)

			Matrix operator+(const Matrix& other) const; //maybe some +=, +=, etc operators would be nice
			Matrix operator+(const float& scalar) const;
			Matrix operator-(const Matrix& other) const;
			Matrix operator*(const Matrix& other) const;
			float& operator()(const int col, const int row); //only c++23 has multi parameter operator[]
			const float& operator()(const int col, const int row) const;
			int getRows() const;
			int getCols() const;
			static Matrix rotationMat2D(float theta);

			int boundsCheck(int row, int col); //private method
	};
}

#endif // PHYSICSIM_MATRIX_HPP
