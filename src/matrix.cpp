// todo: 
//		make protected vars
//		make returning stuff consistent

#include "physicsim/matrix.hpp"
#include <stdexcept>
#include <iostream>

/*! Default constructor for Matrix class
 * sets rows and cols to 0, and arr to nullptr
 *
 * Be careful not to free this with nullptr
 * 
 * TODO: nullptr checks before indexing
 */
physicsim::Matrix::Matrix() {
	this->rows = 0; this->cols = 0;
	this->arr = nullptr;
}

/*! Construct empty matrix with specified rows and columns
 * uninitialised values
 *
 * TODO: change to zeros
 */
physicsim::Matrix::Matrix(int rows, int cols) {
	this->rows = rows; this->cols = cols;
	this->arr = new float[rows * cols];
}

/*! Construct matrix with specified rows and columns, and fill in  with specified values
 * throws error if list does not match dimensions
 */
physicsim::Matrix::Matrix(int rows, int cols, std::initializer_list<float> vals) {
	if(vals.size() != rows * cols) {
		throw std::out_of_range("initialiser list does not match matrix dimensions");
	}
	this->rows = rows; this->cols = cols;
	this->arr = new float[rows * cols];
	for(int i{}; i < rows * cols; i++) {
		this->arr[i] = *(vals.begin() + i); //pointer/iterator magic
	}
}

/*! Swaps 2 rows of a matrix iterating through columns
 *
 * TODO: test to make sure no reference magic
 */
void physicsim::Matrix::swapRows(int row1, int row2) {
	if (row1 < 0 || row1 > this->rows || row2 < 0 || row2 > this->rows) { //change with bounds checks for individual rows
		throw std::invalid_argument("row out of bounds");
	}

	float t;
	for (int i = 0; i < this->cols; i++) {
		t = (*this)(row1, i);
		(*this)(row1, i) = (*this)(row2, i);
		(*this)(row2, i) = t;
	}

}

physicsim::Matrix physicsim::Matrix::transpose() {
	physicsim::Matrix t(this->cols, this->rows);
	for (int i = 0; i < this->rows; i++) {
		for (int j = 0; j < this->cols; j++) {
			t(j, i) = (*this)(i, j);
		}
	}
	return t;
}

physicsim::Matrix physicsim::Matrix::scalarMultiply(float lambda) const {
	physicsim::Matrix t(this->rows, this->cols);
	for (int i = 0; i < this->rows * this->cols; i++) {
		t.arr[i] = this->arr[i] * lambda;
	}
	return t;
}

physicsim::Matrix physicsim::Matrix::scalarDivide(float lambda) const {
	physicsim::Matrix t(this->rows, this->cols);
	for (int i = 0; i < this->rows * this->cols; i++) {
		t.arr[i] = this->arr[i] / lambda;
	}
	return t;
}

/*! Add matrices of equal dimensions
 *
 * Throws error for unequal dimensions
 *
 * TODO: return reference
 */
physicsim::Matrix physicsim::Matrix::operator+(const Matrix& other) const {
	if (this->rows != other.rows || this->cols != other.cols) {
		throw std::invalid_argument("both matrices must have same size");
	}
	physicsim::Matrix t(this->rows, this->cols);

	for (int i = 0; i < this->rows * this->cols; i++) {
		t.arr[i] = this->arr[i]+ other.arr[i];
	}

	return t;
}


/*! Add scalar value to all elements of matrix
 *
 * TODO: return reference
 */
physicsim::Matrix physicsim::Matrix::operator+(const float& scalar) const {
	physicsim::Matrix t(this->rows, this->cols);

	for (int i = 0; i < this->rows * this->cols; i++) {
		t.arr[i] = this->arr[i] + scalar;
	}

	return t;
}

/*! Subtract matrices of equal dimensions
 *
 * Throws error for unequal matrix dimensions
 *
 * TODO: return reference
 */
physicsim::Matrix physicsim::Matrix::operator-(const Matrix& other) const {
	if (this->rows != other.rows || this->cols != other.cols) {
		throw std::invalid_argument("both matrices must have same size");
	}
	physicsim::Matrix t(this->rows, this->cols);

	for (int i = 0; i < this->rows * this->cols; i++) {
		t.arr[i] = this->arr[i] - other.arr[i];
	}

	return t;
}

physicsim::Matrix physicsim::Matrix::operator-(const float& scalar) const {
	physicsim::Matrix t(this->rows, this->cols);

	for (int i = 0; i < this->rows * this->cols; i++) {
		t.arr[i] = this->arr[i] - scalar;
	}

	return t;
}


void physicsim::Matrix::operator+=(const Matrix& other) {
	if (this->rows != other.rows || this->cols != other.cols) {
		throw std::invalid_argument("both matrices must have same size");
	}

	for (int i = 0; i < this->rows * this->cols; i++) {
		this->arr[i] += other.arr[i];
	}
}

void physicsim::Matrix::operator+=(const float& scalar) {
	for (int i = 0; i < this->rows * this->cols; i++) {
		this->arr[i] += scalar;
	}
}

void physicsim::Matrix::operator-=(const Matrix& other) {
	if (this->rows != other.rows || this->cols != other.cols) {
		throw std::invalid_argument("both matrices must have same size");
	}

	for (int i = 0; i < this->rows * this->cols; i++) {
		this->arr[i] -= other.arr[i];
	}
}

void physicsim::Matrix::operator-=(const float& scalar) {
	for (int i = 0; i < this->rows * this->cols; i++) {
		this->arr[i] -= scalar;
	}
}

physicsim::Matrix physicsim::Matrix::operator*(const Matrix& other) const {
	if (this->cols != other.rows) {
		throw std::invalid_argument("left matrix must have same number of columns as left matrix rows");
	}
	physicsim::Matrix t(this->rows, other.cols);
	int runningTot;
	for (int leftRow = 0; leftRow < this->rows; leftRow++) {
		for (int rightCol = 0; rightCol < other.cols; rightCol++) {
			runningTot = 0;
			for (int leftCol = 0; leftCol < this->cols; leftCol++) {
				runningTot += (*this)(leftRow, leftCol) * other(leftCol, rightCol);
			}
			t(leftRow, rightCol) = runningTot;
		}
	}

	return t;
}

/*! Indexing a 2d matrix with syntax m(row, col)
 * throws error for index out of range
 *
 * This version non const, can use to assign new value
 *
 * move to correct function
 */
void physicsim::Matrix::operator=(const physicsim::Matrix& other) {
	this->rows = other.getRows(); this->cols = other.getCols();
	this->arr = new float[this->rows * this->cols];
	std::copy(other.arr, other.arr + this->rows * this->cols, arr);
}

float& physicsim::Matrix::operator()(const int row, const int col) {
	if(!this->indexOutOfBounds(row, col)) {
		return this->arr[row * this->cols + col];
	}
	throw std::out_of_range("Index out of bounds"); //add more personalised error message later
}

/*! Indexing a 2d matrix with syntax m(row, col)
 * throws error for index out of range
 *
 * This version is const, only for copying
 *
 * TODO: version for single column or single row
 */
const float& physicsim::Matrix::operator()(const int row, const int col) const {
	if(!this->indexOutOfBounds(row, col)) {
		return this->arr[row * this->cols + col];
	}
	throw std::out_of_range("Index out of bounds");
}

//! Getter for rows member
int physicsim::Matrix::getRows() const {
	return this->rows;
}

int physicsim::Matrix::colOutOfBounds(int col) const
{
	return 0;
}

float physicsim::Matrix::vectorMagnitude() const {
	if (this->cols > 1) {
		throw std::invalid_argument("Must be vector");
	}
	float res = 0;
	for (int i = 0; i < this->rows; i++) {
		res += this->arr[i] * this->arr[i];
	}
	return std::sqrt(res);
}

float physicsim::Matrix::vectorMagnitudeSqrd() const {
	if (this->cols > 1) {
		throw std::invalid_argument("Must be vector");
	}
	float res = 0;
	for (int i = 0; i < this->rows; i++) {
		res += this->arr[i] * this->arr[i];
	}
	return res;
}

//! Getter for cols member
int physicsim::Matrix::getCols() const {
	return this->cols;
}

physicsim::Matrix physicsim::Matrix::rotationMat2D(float theta)
{
	float cTheta = std::cos(theta); float sTheta = std::sin(theta);
	return physicsim::Matrix(2, 2, { cTheta, -sTheta, sTheta, cTheta });
}

/* Return 0 if index out of bounds, return 1 if index in bounds
 */
int physicsim::Matrix::indexOutOfBounds(int row, int col) const {
	if (this->cols > col || this->rows > row) {
		return 0;
	}
	return 1;
}

/*! Print matrix in 2 for loops
 *
 * TODO: remove loops to lower time complexity
 */
void physicsim::Matrix::print() const {
	for(int i{}; i < this->rows; i++) {
		for(int j{}; j < this->cols; j++) {
			std::cout << (*this)(i, j) << " ";
		}
		std::cout << std::endl;
	}
}

/*physicsim::Matrix::~Matrix() {
	if (arr != nullptr) {
		delete[] arr;
	}
}*/
