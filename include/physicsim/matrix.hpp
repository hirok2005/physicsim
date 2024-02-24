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
		public:
			float* arr; // matrix is stored as 1 contiguous array
			Matrix(); //default constructor, sets rows and cols to zero
			Matrix(int rows, int cols);
			Matrix(int rows, int cols, std::initializer_list<float> vals); //second constructor for when you have all values, consider std::initialiser_list
			
			void swapRows(int row1, int row2);
			Matrix transpose();

			Matrix scalarMultiply(float lambda) const;
			Matrix scalarDivide(float lambda) const; //shorthand for easier usage, essentially just matrix.scalarMultiply(1/lambda)

			Matrix operator+(const Matrix& other) const; //maybe some +=, +=, etc operators would be nice
			Matrix operator+(const float& scalar) const;
			Matrix operator-(const Matrix& other) const;
			Matrix operator-(const float& scalar) const;
			void operator+=(const Matrix& other);
			void operator+=(const float& scalar);
			void operator-=(const Matrix& other);
			void operator-=(const float& scalar);
			Matrix operator*(const Matrix& other) const;
			void operator=(const Matrix& other);
			float& operator()(const int row, const int col); //only c++23 has multi parameter operator[]
			const float& operator()(const int row, const int col) const;
			//maybe an index operator to return a row?

			int getRows() const;
			int getCols() const;
			static Matrix rotationMat2D(float theta);

			int rowOutOfBounds(int row) const;
			int colOutOfBounds(int col) const;
			int indexOutOfBounds(int row, int col) const; //private method
			
			~Matrix() = default;
	};
}

#endif // PHYSICSIM_MATRIX_HPP
