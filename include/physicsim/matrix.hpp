// todo:
//		make int type consistent, lazy
//		sort into private, public

#ifndef PHYSICSIM_MATRIX_HPP
#define PHYSICSIM_MATRIX_HPP

#include <initializer_list>
#include <vector>
#include <cmath>

namespace physicsim {
	class Matrix {
		private:
			int rows; int cols;
		public:
			std::vector<double> arr; // matrix is stored as 1 contiguous array
			Matrix(); //default constructor, sets rows and cols to zero
			Matrix(int rows, int cols);
			Matrix(int rows, int cols, std::initializer_list<double> vals); //second constructor for when you have all values, consider std::initialiser_list
			//make copy constructors
			void swapRows(int row1, int row2); //return reference, change to pointers to rows
			Matrix transpose();
			Matrix scalarMultiply(double lambda) const;
			Matrix scalarDivide(double lambda) const; //shorthand for easier usage, essentially just matrix.scalarMultiply(1/lambda)
			double dot(const Matrix& other) const;
      Matrix project(const Matrix& other, bool isNorm=false) const; // project vector onto other
			Matrix operator+(const Matrix& other) const; //maybe some +=, +=, etc operators would be nice
			Matrix operator+(const double& scalar) const; //all should return references
			Matrix operator-(const Matrix& other) const;
			Matrix operator-(const double& scalar) const;
			Matrix& operator+=(const Matrix& other);
			Matrix& operator+=(const double& scalar);
			Matrix& operator-=(const Matrix& other);
			Matrix& operator-=(const double& scalar);
			Matrix operator*(const Matrix& other) const;
			Matrix& operator=(const Matrix& other);
			bool operator==(const Matrix& other);
			double& operator()(const int row, const int col); //only c++23 has multi parameter operator[]
			const double& operator()(const int row, const int col) const;
			//maybe an index operator to return a row?
			double vectorMagnitudeSqrd() const;
			double vectorMagnitude() const;
			Matrix normalise() const;
      Matrix perp() const;

			int getRows() const;
			int getCols() const;

			int rowOutOfBounds(int row) const;
			int colOutOfBounds(int col) const;
			int indexOutOfBounds(int row, int col) const; //private method
			void print() const;

			~Matrix() = default;
	};

	Matrix rotationMat2D(double theta);
}



#endif // PHYSICSIM_MATRIX_HPP
