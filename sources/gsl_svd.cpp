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

#include <stdexcept>
#include <sstream>
#include <map>

#include "gsl_svd.h"

namespace gsl {

	vector::vector() : ptr_(NULL) {}

 	vector::vector(size_t size) {
 		ptr_ = gsl_vector_calloc(size);
 	}

 	vector::vector(const vector& v) {
 		if (!v.ptr_) {
 			ptr_ = NULL;
 		} else {
 			ptr_ = gsl_vector_alloc(v.ptr_->size);
 			gsl_vector_memcpy(ptr_, v.ptr_);
 		}
 	}

 	vector::vector(const std::vector<float>& vec) {
 		ptr_ = gsl_vector_alloc(vec.size());
 		for (size_t i = 0; i < vec.size(); ++i)
 			this->operator[](i) = vec[i];
 	}

 	vector::~vector() {
 		if (ptr_) gsl_vector_free(ptr_);
 	}

 	vector& vector::operator=(const vector& v) {
 		if (!v.ptr_) {
 			if (ptr_) gsl_vector_free(ptr_);
 		} else if (this != &v) {
 			if (ptr_) gsl_vector_free(ptr_);
 			ptr_ = gsl_vector_alloc(v.ptr_->size);
 			gsl_vector_memcpy(ptr_, v.ptr_);
 		}
 		return *this;
 	}

 	double& vector::operator[](size_t off) {
 		if (!ptr_) throw std::runtime_error("empty gsl::vector");
 		return *gsl_vector_ptr(ptr_, off);
 	}

 	const double& vector::operator[](size_t off) const {
 		if (!ptr_) throw std::runtime_error("empty gsl::vector");
 		return *gsl_vector_const_ptr(ptr_, off);
 	}

 	size_t vector::size() const {
 		return ptr_->size;
 	}

 	matrix::matrix() : ptr_(NULL) {}

 	matrix::matrix(const matrix& m) {
 		if (!m.ptr_) {
 			ptr_ = NULL;
 		} else {
 			ptr_ = gsl_matrix_alloc(m.ptr_->size1, m.ptr_->size2);
 			gsl_matrix_memcpy(ptr_, m.ptr_);
 		}
 	}

 	matrix::matrix(size_t dx, size_t dy) {
 		ptr_ = gsl_matrix_calloc(dx, dy);
 	}

 	matrix::matrix(const std::pair<size_t, size_t>& size) {
 		ptr_ = gsl_matrix_calloc(size.first, size.second);
 	}

 	matrix::matrix(const std::vector<float>& vec, size_t pitch) {
 		ptr_ = gsl_matrix_alloc(vec.size() / pitch, pitch);
 		for (size_t y = 0; y < vec.size() / pitch; ++y) {
 			for (size_t x = 0; x < pitch; ++x) {
 				this->operator()(x, y) = vec[x + (y * pitch)];
 			}
 		}
 	}

 	matrix& matrix::operator=(const matrix& m) {
 		if (!m.ptr_) {
 			if (ptr_) gsl_matrix_free(ptr_);
 		} else if (this != &m) {
			if (ptr_) gsl_matrix_free(ptr_);
			ptr_ = gsl_matrix_alloc(m.ptr_->size1, m.ptr_->size2);
			gsl_matrix_memcpy(ptr_, m.ptr_);
		}
		return *this;
	}

	double& matrix::operator()(size_t x, size_t y) {
		if (!ptr_) throw std::runtime_error("empty gsl::matrix");
		return *gsl_matrix_ptr(ptr_, x, y);
	}

	const double& matrix::operator()(size_t x, size_t y) const {
		if (!ptr_) throw std::runtime_error("empty gsl::matrix");
		return *gsl_matrix_const_ptr(ptr_, x, y);
	}

	std::pair<size_t, size_t> matrix::size() const {
		return std::make_pair<size_t, size_t>(
			ptr_->size1,
			ptr_->size2);
	}

 	matrix::~matrix() {
 		if (ptr_) gsl_matrix_free(ptr_);
 	}

 	matrix transpose(const matrix& m) {
 		matrix t(m.ptr_->size2, m.ptr_->size1);
 		gsl_matrix_transpose_memcpy(t.ptr_, m.ptr_);
 		return t;
 	}

 	matrix operator+(const matrix& m1, const matrix& m2) {
 		matrix out(m1);
 		gsl_matrix_add(out.ptr_, m2.ptr_);
 		return out;
 	}

 	matrix operator-(const matrix& m1, const matrix& m2) {
 		matrix out(m1);
 		gsl_matrix_sub(out.ptr_, m2.ptr_);
 		return out;
 	}

 	matrix operator*(const matrix& m1, const matrix& m2) {
 		if (m1.ptr_->size2 != m2.ptr_->size1) {
 			std::stringstream ss("");
 			ss  << "incompatible matrix m1("
 				<< m1.ptr_->size1 
 				<< ", "
 				<< m1.ptr_->size2
 				<< ") * m2("
 				<< m2.ptr_->size1
 				<< ", "
 				<< m2.ptr_->size2
 				<< ")";
 			throw std::runtime_error(ss.str());
 		}
 		matrix out(m1.ptr_->size1, m2.ptr_->size2);
 		gsl_blas_dgemm(
 			CblasNoTrans, 
 			CblasNoTrans, 
 			1.0, 
 			m1.ptr_, 
 			m2.ptr_, 
 			0.0, 
 			out.ptr_);
 		return out;
 	}

 	matrix operator*(const matrix& m , double x) {
 		matrix out(m);
 		gsl_matrix_scale(out.ptr_, x);
 		return out;
 	}

 	matrix operator+(const matrix& m, double x) {
 		matrix out(m);
 		gsl_matrix_add_constant(out.ptr_, x);
 		return out;
 	}

 	void SVD(
 		matrix& A,
 		matrix& V,
 		vector& S,
 		vector& work) {
 		int ret = gsl_linalg_SV_decomp(
 			A.ptr_,
 			V.ptr_,
 			S.ptr_,
 			work.ptr_);
 	}

 	void SVD_mod(
 		matrix& A,
 		matrix& X,
 		matrix& V,
 		vector& S,
 		vector& work) {
 		int ret = gsl_linalg_SV_decomp_mod(
 			A.ptr_,
 			X.ptr_,
 			V.ptr_,
 			S.ptr_,
 			work.ptr_);
 	}

 	void SVD_jacobi(
 		matrix& A,
 		matrix& V,
 		vector& S) {
 		int ret = gsl_linalg_SV_decomp_jacobi(
 			A.ptr_,
 			V.ptr_,
 			S.ptr_);
 	}

}
