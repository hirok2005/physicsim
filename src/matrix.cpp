// todo: 
//		make protected vars
//		make returning stuff consistent

#include "physicsim/matrix.hpp"
#include <stdexcept>
#include <vector>
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
	this->arr = std::vector<double>(0);
}

/*! Construct empty matrix with specified rows and columns
 * uninitialised values
 *
 * TODO: change to zeros
 */
physicsim::Matrix::Matrix(int rows, int cols) {
	this->rows = rows; this->cols = cols;
	this->arr = std::vector<double>(rows * cols);
}

/*! Construct matrix with specified rows and columns, and fill in  with specified values
 * throws error if list does not match dimensions
 */
physicsim::Matrix::Matrix(int rows, int cols, std::initializer_list<double> vals) {
	if (vals.size() != rows * cols) {
		throw std::out_of_range("initialiser list does not match matrix dimensions");
	}
	this->rows = rows; this->cols = cols;
	this->arr = std::vector<double>(rows * cols);
	for (int i{}; i < rows * cols; i++) {
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

	double t;
	for (int i = 0; i < this->cols; i++) {
		t = (*this)(row1, i);
		(*this)(row1, i) = (*this)(row2, i);
		(*this)(row2, i) = t;
	}

}

/*! Transpose the matrix
 *
 * TODO: testing
 */
physicsim::Matrix physicsim::Matrix::transpose() {
	physicsim::Matrix t(this->cols, this->rows);
	for (int i = 0; i < this->rows; i++) {
		for (int j = 0; j < this->cols; j++) {
			t(j, i) = (*this)(i, j);
		}
	}
	return t;
}

/*! Multiply matrix by scalar
 *
 * TODO: testing
 */
physicsim::Matrix physicsim::Matrix::scalarMultiply(double lambda) const {
	physicsim::Matrix t(this->rows, this->cols);
	for (int i = 0; i < this->rows * this->cols; i++) {
		t.arr[i] = this->arr[i] * lambda;
	}
	return t;
}

/*! Divide matrix by scalar
 * TODO: testing
 */
physicsim::Matrix physicsim::Matrix::scalarDivide(double lambda) const {
	physicsim::Matrix t(this->rows, this->cols);
	for (int i = 0; i < this->rows * this->cols; i++) {
		t.arr[i] = this->arr[i] / lambda;
	}
	return t;
}

/*!
 * project vector onto other vector using standard projection formula
 */
physicsim::Matrix physicsim::Matrix::project(const physicsim::Matrix& other, bool isNorm) const {
	double scalar = this->dot(other);
	if (!isNorm) {
		scalar /= other.dot(other);
	}
	return other.scalarMultiply(scalar);
}

/*! Dot product of 2 matrices. Errors when matrices not same size.
 *
 * TODO: require that matrices are vectors
 */
double physicsim::Matrix::dot(const Matrix& other) const {
	if (this->cols > 1 || this->cols != other.cols || this->rows != other.rows) {
		throw std::invalid_argument("must be vectors of same size");
	}
	double res = 0;
	for (int i = 0; i < this->rows; i++) {
		res += this->arr[i] * other.arr[i];
	}
	return res;
}

/*! Add matrices of equal dimensions
 *
 * Throws error for unequal dimensions
 */
physicsim::Matrix physicsim::Matrix::operator+(const Matrix& other) const {
	if (this->rows != other.rows || this->cols != other.cols) {
		throw std::invalid_argument("both matrices must have same size");
	}
	physicsim::Matrix t(this->rows, this->cols);

	for (int i = 0; i < this->rows * this->cols; i++) {
		t.arr[i] = this->arr[i] + other.arr[i];
	}

	return t;
}


/*! Add scalar value to all elements of matrix
 *
 * TODO: return reference
 */
physicsim::Matrix physicsim::Matrix::operator+(const double& scalar) const {
	physicsim::Matrix t(this->rows, this->cols);

	for (int i = 0; i < this->rows * this->cols; i++) {
		t.arr[i] = this->arr[i] + scalar;
	}

	return t;
}

/*! Subtract matrices of equal dimensions
 *
 * Throws error for unequal matrix dimensions
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

/*! Subtract scalar from all elements of matrix
 *
 * TODO: testing
 */
physicsim::Matrix physicsim::Matrix::operator-(const double& scalar) const {
	physicsim::Matrix t(this->rows, this->cols);

	for (int i = 0; i < this->rows * this->cols; i++) {
		t.arr[i] = this->arr[i] - scalar;
	}

	return t;
}

/*! Reassigns matrix adding with other matrix
 */
physicsim::Matrix& physicsim::Matrix::operator+=(const Matrix& other) {
	if (this->rows != other.rows || this->cols != other.cols) {
		throw std::invalid_argument("both matrices must have same size");
	}

	for (int i = 0; i < this->rows * this->cols; i++) {
		this->arr[i] += other.arr[i];
	}
	return *this;
}

/*! Reassigns matrix with added scalar
 */
physicsim::Matrix& physicsim::Matrix::operator+=(const double& scalar) {
	for (int i = 0; i < this->rows * this->cols; i++) {
		this->arr[i] += scalar;
	}
	return *this;
}

/*! Reassigns matrix with subtracted other matrix
 */
physicsim::Matrix& physicsim::Matrix::operator-=(const Matrix& other) {
	if (this->rows != other.rows || this->cols != other.cols) {
		throw std::invalid_argument("both matrices must have same size");
	}

	for (int i = 0; i < this->rows * this->cols; i++) {
		this->arr[i] -= other.arr[i];
	}
	return *this;
}

/*! Reassigns matrix with subtracted scalar
 */
physicsim::Matrix& physicsim::Matrix::operator-=(const double& scalar) {
	for (int i = 0; i < this->rows * this->cols; i++) {
		this->arr[i] -= scalar;
	}
	return *this;
}

/*! Matrix multiplication operator
 */
physicsim::Matrix physicsim::Matrix::operator*(const Matrix& other) const {
	if (this->cols != other.rows) {
		throw std::invalid_argument("left matrix must have same number of columns as left matrix rows");
	}
	physicsim::Matrix t(this->rows, other.cols);
	double runningTot;
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
physicsim::Matrix& physicsim::Matrix::operator=(const physicsim::Matrix& other) {
	this->rows = other.getRows(); this->cols = other.getCols();
	// this->arr = new double[this->rows * this->cols];
	// std::copy(other.arr, other.arr + this->rows * this->cols, arr);
	this->arr = other.arr;
	return *this;
}

bool physicsim::Matrix::operator==(const physicsim::Matrix& other) {
    if (other.getRows() != this->rows || other.getCols() != this->cols) {
		return false;
	}
	for (int i = 0; i < this->rows * this->cols; i++) {
		if (this->arr[i] != other.arr[i]) {
			return false;
		}
	}
}

double& physicsim::Matrix::operator()(const int row, const int col) {
	if (!this->indexOutOfBounds(row, col)) {
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
const double& physicsim::Matrix::operator()(const int row, const int col) const {
	if (!this->indexOutOfBounds(row, col)) {
		return this->arr[row * this->cols + col];
	}
	throw std::out_of_range("Index out of bounds");
}

//! Getter for rows member
int physicsim::Matrix::getRows() const {
	return this->rows;
}

/*! Returns 1 if the given column is greater than what can be indexed
 */
int physicsim::Matrix::colOutOfBounds(int col) const
{
	if (this->cols > col || col < 0) {
		return 1;
	}
	return 0;
}

/*! Calculates vector magnitude using pythagoras theorem
 */
double physicsim::Matrix::vectorMagnitude() const {
	if (this->colOutOfBounds(1)) {
		throw std::invalid_argument("Must be vector");
	}
	double res = this->dot(*this);
	return std::sqrt(res);
}

/*! Calculates vector magnitude squared
 */
double physicsim::Matrix::vectorMagnitudeSqrd() const {
	if (this->colOutOfBounds(1)) {
		throw std::invalid_argument("Must be vector");
	}
	return this->dot(*this);
}

physicsim::Matrix physicsim::Matrix::normalise() const {
	if (this->colOutOfBounds(1)) {
		throw std::invalid_argument("Must be vector");
	}
	physicsim::Matrix t = *this;
	double distanceInv = 1 / this->vectorMagnitude();
	t = t.scalarMultiply(distanceInv);
	return t;
}


/*! Returns the perpendicular of vector
 * */
physicsim::Matrix physicsim::Matrix::perp() const {
	// TODO: only for 2d vector right now (all we will need) can implement for n-dimensional later
	if (this->colOutOfBounds(1)) {
		throw std::invalid_argument("Must be vector");
	}
	return physicsim::Matrix(2, 1, { -this->arr[1], this->arr[0] });
}

//! Getter for cols member
int physicsim::Matrix::getCols() const {
	return this->cols;
}

/*! Rotates matrix by theta radians
 */
physicsim::Matrix physicsim::rotationMat2D(double theta)
{
	double cTheta = std::cos(theta); double sTheta = std::sin(theta);
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
	for (int i{}; i < this->rows; i++) {
		for (int j{}; j < this->cols; j++) {
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
