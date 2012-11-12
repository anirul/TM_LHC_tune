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

#include "gsl_svd.h"

namespace gsl {

	void linalg_SV_decomp(
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

	void linalg_SV_decomp_mod(
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

	void linalg_SV_decomp_jacobi(
		matrix& A,
		matrix& V,
		vector& S) {
		int ret = gsl_linalg_SV_decomp_jacobi(
			A.ptr_,
			V.ptr_,
			S.ptr_);
	}

	void linalg_SV_solve(
		const matrix& U, 
		const matrix& V,
		const vector& S,
		const vector& b,
		vector& x) {
		int ret = gsl_linalg_SV_solve(
			U.ptr_,
			V.ptr_,
			S.ptr_,
			b.ptr_,
			x.ptr_);
	}

}