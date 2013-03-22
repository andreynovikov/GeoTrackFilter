/*
(The MIT License.)

Copyright (c) 2009, Kevin Lacker.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

/*
 * This is a Java port of original C code by Kevin Lacker.
 * https://github.com/lacker/ikalman
 * 
 * Ported by Andrey Novikov, 2013
 */

package com.androzic.utils;

public class Matrix {
	/* Dimensions */
	int rows;
	int cols;

	/* Contents of the matrix */
	double[][] data;

	/*
	 * Allocate memory for a new matrix. Zeros out the matrix. Assert-fails if
	 * we are out of memory.
	 */
	Matrix(int rows, int cols) {
		this.rows = rows;
		this.cols = cols;
		this.data = new double[rows][cols];
	}

	/* Set values of a matrix, row by row. */
	void set_matrix(double... arg) {
		assert (arg.length == rows * cols);
		for (int i = 0; i < rows; i++) {
			for (int j = 0; j < cols; j++) {
				data[i][j] = arg[i * cols + j];
			}
		}
	}

	/* Turn m into an identity matrix. */
	void set_identity_matrix() {
		assert (rows == cols);
		for (int i = 0; i < rows; ++i) {
			for (int j = 0; j < cols; ++j) {
				if (i == j) {
					data[i][j] = 1.0;
				} else {
					data[i][j] = 0.0;
				}
			}
		}
	}

	/* Copy a matrix. */
	static void copy_matrix(Matrix source, Matrix destination) {
		assert (source.rows == destination.rows);
		assert (source.cols == destination.cols);
		for (int i = 0; i < source.rows; ++i) {
			for (int j = 0; j < source.cols; ++j) {
				destination.data[i][j] = source.data[i][j];
			}
		}
	}

	/* Pretty-print a matrix. */
	void print_matrix() {
		for (int i = 0; i < rows; ++i) {
			for (int j = 0; j < cols; ++j) {
				if (j > 0)
					System.out.print(" ");
				System.out.format("%6.2f", data[i][j]);
			}
			System.out.print("\n");
		}
	}

	/* Add matrices a and b and put the result in c. */
	static void add_matrix(Matrix a, Matrix b, Matrix c) {
		assert (a.rows == b.rows);
		assert (a.rows == c.rows);
		assert (a.cols == b.cols);
		assert (a.cols == c.cols);
		for (int i = 0; i < a.rows; ++i) {
			for (int j = 0; j < a.cols; ++j) {
				c.data[i][j] = a.data[i][j] + b.data[i][j];
			}
		}
	}

	/* Subtract matrices a and b and put the result in c. */
	static void subtract_matrix(Matrix a, Matrix b, Matrix c) {
		assert (a.rows == b.rows);
		assert (a.rows == c.rows);
		assert (a.cols == b.cols);
		assert (a.cols == c.cols);
		for (int i = 0; i < a.rows; ++i) {
			for (int j = 0; j < a.cols; ++j) {
				c.data[i][j] = a.data[i][j] - b.data[i][j];
			}
		}
	}

	/* Subtract from the identity matrix in place. */
	static void subtract_from_identity_matrix(Matrix a) {
		assert (a.rows == a.cols);
		for (int i = 0; i < a.rows; ++i) {
			for (int j = 0; j < a.cols; ++j) {
				if (i == j) {
					a.data[i][j] = 1.0 - a.data[i][j];
				} else {
					a.data[i][j] = 0.0 - a.data[i][j];
				}
			}
		}
	}

	/* Multiply matrices a and b and put the result in c. */
	static void multiply_matrix(Matrix a, Matrix b, Matrix c) {
		assert (a.cols == b.rows);
		assert (a.rows == c.rows);
		assert (b.cols == c.cols);
		for (int i = 0; i < c.rows; ++i) {
			for (int j = 0; j < c.cols; ++j) {
				/*
				 * Calculate element c.data[i][j] via a dot product of one row
				 * of a with one column of b
				 */
				c.data[i][j] = 0.0;
				for (int k = 0; k < a.cols; ++k) {
					c.data[i][j] += a.data[i][k] * b.data[k][j];
				}
			}
		}
	}

	/* Multiply matrix a by b-transpose and put the result in c. */
	/*
	 * This is multiplying a by b-tranpose so it is like multiply_matrix but
	 * references to b reverse rows and cols.
	 */
	static void multiply_by_transpose_matrix(Matrix a, Matrix b, Matrix c) {
		assert (a.cols == b.cols);
		assert (a.rows == c.rows);
		assert (b.rows == c.cols);
		for (int i = 0; i < c.rows; ++i) {
			for (int j = 0; j < c.cols; ++j) {
				/*
				 * Calculate element c.data[i][j] via a dot product of one row
				 * of a with one row of b
				 */
				c.data[i][j] = 0.0;
				for (int k = 0; k < a.cols; ++k) {
					c.data[i][j] += a.data[i][k] * b.data[j][k];
				}
			}
		}
	}

	/* Transpose input and put the result in output. */
	static void transpose_matrix(Matrix input, Matrix output) {
		assert (input.rows == output.cols);
		assert (input.cols == output.rows);
		for (int i = 0; i < input.rows; ++i) {
			for (int j = 0; j < input.cols; ++j) {
				output.data[j][i] = input.data[i][j];
			}
		}
	}

	/* Whether two matrices are approximately equal. */
	static boolean equal_matrix(Matrix a, Matrix b, double tolerance) {
		assert (a.rows == b.rows);
		assert (a.cols == b.cols);
		for (int i = 0; i < a.rows; ++i) {
			for (int j = 0; j < a.cols; ++j) {
				if (Math.abs(a.data[i][j] - b.data[i][j]) > tolerance)
					return false;
			}
		}
		return true;
	}

	/* Multiply a matrix by a scalar. */
	void scale_matrix(double scalar) {
		assert (scalar != 0.0);
		for (int i = 0; i < rows; ++i) {
			for (int j = 0; j < cols; ++j) {
				data[i][j] *= scalar;
			}
		}
	}

	/*
	 * Swap rows r1 and r2 of a matrix. This is one of the three
	 * "elementary row operations".
	 */
	void swap_rows(int r1, int r2) {
		assert (r1 != r2);
		double[] tmp = data[r1];
		data[r1] = data[r2];
		data[r2] = tmp;
	}

	/*
	 * Multiply row r of a matrix by a scalar. This is one of the three
	 * "elementary row operations".
	 */
	void scale_row(int r, double scalar) {
		assert (scalar != 0.0);
		for (int i = 0; i < cols; ++i)
			data[r][i] *= scalar;
	}

	/*
	 * Add a multiple of row r2 to row r1. Also known as a "shear" operation.
	 * This is one of the three "elementary row operations".
	 */
	/* Add scalar * row r2 to row r1. */
	void shear_row(int r1, int r2, double scalar) {
		assert (r1 != r2);
		for (int i = 0; i < cols; ++i)
			data[r1][i] += scalar * data[r2][i];
	}

	/*
	 * Invert a square matrix. Returns whether the matrix is invertible. input
	 * is mutated as well by this routine.
	 */

	/*
	 * Uses Gauss-Jordan elimination.
	 * 
	 * The elimination procedure works by applying elementary row operations to
	 * our input matrix until the input matrix is reduced to the identity
	 * matrix. Simultaneously, we apply the same elementary row operations to a
	 * separate identity matrix to produce the inverse matrix. If this makes no
	 * sense, read wikipedia on Gauss-Jordan elimination.
	 * 
	 * This is not the fastest way to invert matrices, so this is quite possibly
	 * the bottleneck.
	 */
	static boolean destructive_invert_matrix(Matrix input, Matrix output) {
		assert (input.rows == input.cols);
		assert (input.rows == output.rows);
		assert (input.rows == output.cols);

		output.set_identity_matrix();

		/*
		 * Convert input to the identity matrix via elementary row operations.
		 * The ith pass through this loop turns the element at i,i to a 1 and
		 * turns all other elements in column i to a 0.
		 */
		for (int i = 0; i < input.rows; ++i) {
			if (input.data[i][i] == 0.0) {
				/* We must swap rows to get a nonzero diagonal element. */
				int r;
				for (r = i + 1; r < input.rows; ++r) {
					if (input.data[r][i] != 0.0)
						break;
				}
				if (r == input.rows) {
					/*
					 * Every remaining element in this column is zero, so this
					 * matrix cannot be inverted.
					 */
					return false;
				}
				input.swap_rows(i, r);
				output.swap_rows(i, r);
			}

			/*
			 * Scale this row to ensure a 1 along the diagonal. We might need to
			 * worry about overflow from a huge scalar here.
			 */
			double scalar = 1.0 / input.data[i][i];
			input.scale_row(i, scalar);
			output.scale_row(i, scalar);

			/* Zero out the other elements in this column. */
			for (int j = 0; j < input.rows; ++j) {
				if (i == j)
					continue;
				double shear_needed = -input.data[j][i];
				input.shear_row(j, i, shear_needed);
				output.shear_row(j, i, shear_needed);
			}
		}

		return true;
	}
}
