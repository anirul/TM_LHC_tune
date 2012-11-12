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

namespace gsl {

	class vector {
		public:
			gsl_vector* ptr_;
		public:
			vector(size_t size) {
				ptr_ = gsl_vector_calloc(size);
			}
			vector(const std::vector<float>& vec) {
				ptr_ = gsl_vector_alloc(vec.size());
				for (size_t i = 0; i < vec.size(); ++i)
					this->set(i, vec[i]);
			}
			virtual ~vector() {
				gsl_vector_free(ptr_);
			}
		public:
			float get(size_t index) const {
				return gsl_vector_get(ptr_, index);
			}
			void set(size_t index, float val) {
				gsl_vector_set(ptr_, index, val);
			}
	};

	class matrix {
		public:
			gsl_matrix* ptr_;
		public:
			matrix(size_t s1, size_t s2) {
				ptr_ = gsl_matrix_calloc(s1, s2);
			}
			matrix(const std::vector<float>& vec, size_t pitch) {
				ptr_ = gsl_matrix_alloc(pitch, vec.size() / pitch);
				for (size_t y = 0; y < vec.size() / pitch; ++y)
					for (size_t x = 0; x < pitch; ++x)
						this->set(x, y, vec[x + (y * pitch)]);
			}
			virtual ~matrix() {
				gsl_matrix_free(ptr_);
			}
		public:
			float get(size_t i1, size_t i2) const {
				return gsl_matrix_get(ptr_, i1, i2);
			}
			void set(size_t i1, size_t i2, float val) {
				gsl_matrix_set(ptr_, i1, i2, val);
			}
	};

	void linalg_SV_decomp(
		matrix& A,
		matrix& V,
		vector& S,
		vector& work);
	void linalg_SV_decomp_mod(
		matrix& A,
		matrix& X,
		matrix& V,
		vector& S,
		vector& work);
	void linalg_SV_decomp_jacobi(
		matrix& A,
		matrix& V,
		vector& S);
	void linalg_SV_solve(
		const matrix& U, 
		const matrix& V,
		const vector& S,
		const vector& b,
		vector& x);

}

#endif // gsl_svd_HEADER_DEFINED