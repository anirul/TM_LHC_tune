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

#include "fftw_fft.h"

boost::mutex fftwf_fft::lock_;
boost::mutex fftwd_fft::lock_;

time_duration fftwf_fft::run_single(
		std::vector<std::complex<float> >& in_out)
{
	ptime before;
	ptime after;
	before = microsec_clock::universal_time();
	if (in_out.size() <= 0)
		throw std::runtime_error("unvalid buffer size");
	fftw_buffer_in_ = in_out;
	fftw_buffer_out_.resize(in_out.size());
	{
		boost::mutex::scoped_lock lock_it(lock_);
		fftwf_plan plan;
		plan = fftwf_plan_dft_1d(
				fftw_buffer_in_.size(),
				(fftwf_complex*)&fftw_buffer_in_[0],
				(fftwf_complex*)&fftw_buffer_out_[0],
				FFTW_FORWARD,
				FFTW_ESTIMATE);
		fftwf_execute(plan);
		fftwf_destroy_plan(plan);
	}
	in_out = fftw_buffer_out_;
	after = microsec_clock::universal_time();
	return after - before;
}

time_duration fftwd_fft::run_single(
		std::vector<std::complex<double> >& in_out)
{
	ptime before;
	ptime after;
	before = microsec_clock::universal_time();
	if (in_out.size() <= 0)
		throw std::runtime_error("unvalid buffer size");
	fftw_buffer_in_ = in_out;
	fftw_buffer_out_.resize(in_out.size());
	{
		boost::mutex::scoped_lock lock_it(lock_);
		fftw_plan plan;
		plan = fftw_plan_dft_1d(
				fftw_buffer_in_.size(),
				(fftw_complex*)&fftw_buffer_in_[0],
				(fftw_complex*)&fftw_buffer_out_[0],
				FFTW_FORWARD,
				FFTW_ESTIMATE);
		fftw_execute(plan);
		fftw_destroy_plan(plan);
	}
	in_out = fftw_buffer_out_;
	after = microsec_clock::universal_time();
	return after - before;
}

time_duration fftwf_fft::run_multiple(
		std::vector<std::complex<float> >& in_out,
		size_t sub_vec_count)
{
	time_duration total = minutes(0);
	for (int i = 0; i < sub_vec_count; ++i) {
		std::vector<std::complex<float> > sub_vec;
		size_t sub_vec_size = in_out.size() / sub_vec_count;
		sub_vec.resize(sub_vec_size);
		memcpy(
				&sub_vec[0],
				&in_out[i * sub_vec_size],
				sub_vec_size * sizeof(std::complex<float>));
		total += run_single(sub_vec);
		memcpy(
				&in_out[i * sub_vec_size],
				&sub_vec[0],
				sub_vec_size * sizeof(std::complex<float>));
	}
	return total;
}

time_duration fftwd_fft::run_multiple(
		std::vector<std::complex<double> >& in_out,
		size_t sub_vec_count)
{
	time_duration total = minutes(0);
	for (int i = 0; i < sub_vec_count; ++i) {
		std::vector<std::complex<double> > sub_vec;
		size_t sub_vec_size = in_out.size() / sub_vec_count;
		sub_vec.resize(sub_vec_size);
		memcpy(
				&sub_vec[0],
				&in_out[i * sub_vec_size],
				sub_vec_size * sizeof(std::complex<double>));
		total += run_single(sub_vec);
		memcpy(
				&in_out[i * sub_vec_size],
				&sub_vec[0],
				sub_vec_size * sizeof(std::complex<double>));
	}
	return total;
}


