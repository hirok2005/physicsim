// todo: 
//		make protected vars
//		make returning stuff consistent

#include "physicsim/matrix.hpp"

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
		throw _matherr;
	}
	physicsim::Matrix t(this->rows, this->cols);

	for (int i = 0; i < this->rows; i++) {
		for (int j = 0; j < this->cols; j++) {
			t.insert(i, j, this->retrieve(i, j) + other.retrieve(i, j));
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
