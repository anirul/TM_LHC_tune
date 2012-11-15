/*
 * Copyright (c) 2012, Frederic DUBOUCHET
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the CERN nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Frederic DUBOUCHET ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Frederic DUBOUCHET BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef gsl_svd_HEADER_DEFINED
#define gsl_svd_HEADER_DEFINED

#include <vector>

#include <gsl/gsl_math.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>

namespace gsl {

	class vector {
		public:
			gsl_vector* ptr_;
		public:
			vector();
			vector(const vector& v);
			vector(size_t size);
			vector(const std::vector<float>& vec);
			virtual ~vector();
		public:
			vector& operator=(const vector& v);
			double& operator[](size_t off);
			const double& operator[](size_t off) const;
			size_t size() const;
	};

	class matrix {
		public:
			gsl_matrix* ptr_;
		public:
			matrix();
			matrix(const matrix& m);
			matrix(size_t dy, size_t dx);
			matrix(const std::pair<size_t, size_t>& size);
			matrix(const std::vector<float>& vec, size_t pitch);
			virtual ~matrix();
		public:
			matrix& operator=(const matrix& m);
			double& operator()(size_t y, size_t x);
			const double& operator()(size_t y, size_t x) const;
			std::pair<size_t, size_t> size() const;
	};

	// matrix operators
	matrix transpose(const matrix& m);
	matrix operator+(const matrix& m1, const matrix& m2);
	matrix operator-(const matrix& m1, const matrix& m2);
	matrix operator*(const matrix& m1, const matrix& m2);
	matrix operator*(const matrix& m, const double x);
	matrix operator+(const matrix& m, const double x);

	// A = U S V^T
	// A is U after call
	void SVD(
		matrix& A,
		matrix& V,
		vector& S,
		vector& work);
	void SVD_mod(
		matrix& A,
		matrix& X,
		matrix& V,
		vector& S,
		vector& work);
	void SVD_jacobi(
		matrix& A,
		matrix& V,
		vector& S);

}

#endif // gsl_svd_HEADER_DEFINED
