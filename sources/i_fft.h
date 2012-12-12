/*
 * Copyright (c) 2012, Frederic Dubouchet
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
 * THIS SOFTWARE IS PROVIDED BY Frederic Dubouchet ``AS IS'' AND ANY
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

#ifndef i_fft_HEADER_DEFINED
#define i_fft_HEADER_DEFINED

#include <complex>

struct i_fft_f {
	i_fft_f() {}
	virtual ~i_fft_f() {}
	virtual boost::posix_time::time_duration run_single(
			std::vector<std::complex<float> >& in_out) = 0;
	virtual boost::posix_time::time_duration run_multiple(
			std::vector<std::complex<float> >& in_out,
			size_t sub_vec) = 0;
};

struct i_fft_d {
	i_fft_d() {}
	virtual ~i_fft_d() {}
	virtual boost::posix_time::time_duration run_single(
			std::vector<std::complex<double> >& in_out) = 0;
	virtual boost::posix_time::time_duration run_multiple(
			std::vector<std::complex<double> >& int_out,
			size_t sub_vec) = 0;
};

#endif // i_fft_HEADER_DEFINED
