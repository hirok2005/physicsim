// todo: 
//		make protected vars
//		make returning stuff consistent

#include "physicsim/matrix.hpp"
#include <stdexcept>

physicsim::Matrix::Matrix() {
	this->rows = 0; this->cols = 0;
	this->arr = nullptr;
}

physicsim::Matrix::Matrix(int rows, int cols) {
	this->rows = rows; this->cols = cols;
	this->arr = new float[rows * cols];
}

void physicsim::Matrix::insert(int row, int col, float val) {
	this->arr[row * this->cols + col] = val;
}

float physicsim::Matrix::retrieve(int row, int col) const{
	return this->arr[row * this->cols + col];
}
 
void physicsim::Matrix::swapRows(int row1, int row2) {
	if (row1 < 1 || row1 > this->rows || row2 < 1 || row2 > this->rows) {
		throw std::invalid_argument("row out of bounds");
	}

	float t;
	for (int i = 0; i < this->cols; i++) {
		t = this->retrieve(row1, i);
		this->insert(row1, i, this->retrieve(rows, i));
		this->insert(row2, i, t);
	}

}

physicsim::Matrix physicsim::Matrix::transpose() {
	physicsim::Matrix t(this->cols, this->rows);
	for (int i = 0; i < this->rows; i++) {
		for (int j = 0; j < this->cols; j++) {
			t.insert(j, i, this->retrieve(i, j));
		}
	}
	return t;
}

physicsim::Matrix physicsim::Matrix::operator+(const Matrix& other) const {
	if (this->rows != other.rows || this->cols != other.cols) {
		throw std::invalid_argument("both matrices must have same size");
	}
	physicsim::Matrix t(this->rows, this->cols);

	for (int i = 0; i < this->rows; i++) {
		for (int j = 0; j < this->cols; j++) {
			t.insert(i, j, this->retrieve(i, j) + other.retrieve(i, j));
		}
	}

	return t;
}

physicsim::Matrix physicsim::Matrix::operator-(const Matrix& other) const {
	if (this->rows != other.rows || this->cols != other.cols) {
		throw std::invalid_argument("both matrices must have same size");
	}
	physicsim::Matrix t(this->rows, this->cols);

	for (int i = 0; i < this->rows; i++) {
		for (int j = 0; j < this->cols; j++) {
			t.insert(i, j, this->retrieve(i, j) - other.retrieve(i, j));
		}
	}

	return t;
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
				runningTot += this->retrieve(leftRow, leftCol) * other.retrieve(leftCol, rightCol);
			}
			t.insert(leftRow, rightCol, runningTot);
		}
	}

	return t;
}
