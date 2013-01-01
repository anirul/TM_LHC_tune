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

#ifndef CL_fft_HEADER_DEFINED
#define CL_fft_HEADER_DEFINED

#define __CL_ENABLE_EXCEPTIONS
#include <CL/cl.hpp>
#include "i_fft.h"

class cl_fft : public i_fft_f {
private :
	bool pipeline_;
	cl::Buffer cl_buffer_in_x_;
	cl::Buffer cl_buffer_out_y_;
	cl::Buffer cl_buffer_acc_;
	cl::Buffer cl_buffer_short_;
	size_t data_size_;
	size_t sub_vec_size_;
	unsigned int platform_used_;
	unsigned int device_used_;
	std::vector<cl::Device> devices_;
	cl::Context context_;
	cl::CommandQueue queue_;
	cl::Program program_;
	cl::Kernel kernel_fft_;
	cl::Kernel kernel_acc_;
	cl::Kernel kernel_prepare_;
	// debugging variables
	cl_int err_;
	cl::Event event_;
public :
	// subroutines for the run_multiple
	boost::posix_time::time_duration cpu2gpu(
			const std::vector<short>& in_short,
			const std::vector<std::complex<float> >& vec_out);
	boost::posix_time::time_duration run_prepare();
	boost::posix_time::time_duration run_fft(size_t sub_vec);
	boost::posix_time::time_duration run_acc(size_t sub_vec);
	boost::posix_time::time_duration gpu2cpu(std::vector<std::complex<float> >& vec_out);
public :
	cl_fft(bool pipeline = true, bool cl_cpu = false);
	virtual ~cl_fft() {}
	// run a single fft
	virtual boost::posix_time::time_duration run_single(
			std::vector<std::complex<float> >& in_out);
	virtual boost::posix_time::time_duration run_multiple(
			std::vector<std::complex<float> >& in_out,
			size_t sub_vec);
};

#endif // CL_fft_HEADER_DEFINED

