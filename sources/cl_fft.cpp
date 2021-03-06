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

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#define __CL_ENABLE_EXCEPTIONS
#include <CL/cl.hpp>
#ifdef __linux__
#include <GL/glx.h>
#endif
#include <boost/date_time/posix_time/posix_time.hpp>

#include "cl_fft.h"
#include "cl_util.h"

using namespace boost::posix_time;

cl_fft::cl_fft(bool pipeline, bool cl_cpu) : pipeline_(pipeline) {
	//setup devices_ and context_
	std::vector<cl::Platform> platforms;
	err_ = cl::Platform::get(&platforms);
	device_used_ = 0;
	data_size_ = 0;
	bool device_found = false;
	for (platform_used_ = 0; (platform_used_ < platforms.size()) && !device_found; ++platform_used_) {
		try {
			err_ = platforms[platform_used_].getDevices((cl_cpu) ? CL_DEVICE_TYPE_CPU : CL_DEVICE_TYPE_GPU, &devices_);
			int t = devices_.front().getInfo<CL_DEVICE_TYPE>();
			cl_context_properties properties[] = {
					CL_CONTEXT_PLATFORM,
					(cl_context_properties)(platforms[platform_used_])(),
					0
			};
			context_ = cl::Context((cl_cpu) ? CL_DEVICE_TYPE_CPU : CL_DEVICE_TYPE_GPU, properties);
			devices_ = context_.getInfo<CL_CONTEXT_DEVICES>();
			std::cout << "Info            : selected device on platform (" << platform_used_ << ")" << std::endl;
			device_found = true;
		} catch (const cl::Error& err) {
			std::cerr << "Warning         : no suitable device on platform (" << platform_used_ << ")" << std::endl;
		}
	}
	if (!device_found) throw std::runtime_error("could not find a valid device!");
	queue_ = cl::CommandQueue(context_, devices_[device_used_], 0, &err_);
	std::cout << "device name     : " << devices_[device_used_].getInfo<CL_DEVICE_NAME>() << std::endl;
	std::ifstream ifs("./fft.cl");
	std::string kernel_source(
			(std::istreambuf_iterator<char>(ifs)),
			std::istreambuf_iterator<char>());
	ifs.close();
	cl::Program::Sources source(
			1,
			std::make_pair(kernel_source.c_str(),
					kernel_source.size()));
	program_ = cl::Program(context_, source);
	try {
		err_ = program_.build(devices_);
	} catch (const cl::Error& er) {
		std::cerr << "build status    : "
				<< program_.getBuildInfo<CL_PROGRAM_BUILD_STATUS>(devices_[0]) << std::endl;
		std::cerr << "build options   : "
				<< program_.getBuildInfo<CL_PROGRAM_BUILD_OPTIONS>(devices_[0]) << std::endl;
		std::cerr << "build log       : "  << std::endl
				<< program_.getBuildInfo<CL_PROGRAM_BUILD_LOG>(devices_[0]) << std::endl;
		throw er;
	}
}

time_duration cl_fft::run_single(
		std::vector<std::complex<float> >& in_out)
{
	ptime before;
	ptime after;
	before = microsec_clock::universal_time();
	data_size_ = in_out.size();
	kernel_fft_ = cl::Kernel(program_, "fftRadix2Kernel", &err_);
	std::vector<cl_float2>::const_iterator ite;
	//initialize our CPU memory arrays, send them to the device and set the kernel_ arguements
	cl_buffer_in_x_ = cl::Buffer(
			context_,
			CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
			sizeof(cl_float2) * data_size_,
			(void*)&in_out[0],
			&err_);
	cl_buffer_out_y_ = cl::Buffer(
			context_,
			CL_MEM_READ_WRITE,
			sizeof(cl_float2) * data_size_,
			NULL,
			&err_);
	queue_.finish();
	//set the arguments of our kernel_
	err_ = kernel_fft_.setArg(0, cl_buffer_in_x_);
	err_ = kernel_fft_.setArg(1, cl_buffer_out_y_);
	//Wait for the command queue_ to finish these commands before proceeding
	queue_.finish();
	double p = log2(data_size_);
	for (int i = 1; i <= (data_size_ >> 1); i *= 2) {
		// enqueue the new parameter p
		err_ = kernel_fft_.setArg(2, i);
		// make the computation
		err_ = queue_.enqueueNDRangeKernel(
				kernel_fft_,
				cl::NullRange,
				cl::NDRange(data_size_ >> 1, 1),
				cl::NullRange,
				NULL,
				&event_);
		queue_.finish();

		if (i != (data_size_ >> 1)) {
			err_ = queue_.enqueueCopyBuffer(
					cl_buffer_out_y_,
					cl_buffer_in_x_,
					0,
					0,
					data_size_ * sizeof(cl_float2),
					NULL,
					&event_);
			queue_.finish();
		}
	}
	err_ = queue_.enqueueReadBuffer(
			cl_buffer_out_y_,
			CL_TRUE,
			0,
			data_size_ * sizeof(cl_float2),
			&in_out[0],
			NULL,
			&event_);
	queue_.finish();
	after = microsec_clock::universal_time();
	return after - before;
}

time_duration cl_fft::cpu2gpu(
		const std::vector<short>& in_short,
		const std::vector<float>& vec_out)
{
	ptime before = microsec_clock::universal_time();
	//initialize our CPU memory arrays, send them to the device and set the kernel_ arguments
	cl_buffer_in_x_ = cl::Buffer(
			context_,
			CL_MEM_READ_WRITE,
			sizeof(cl_float2) * data_size_,
			NULL,
			&err_);
	cl_buffer_out_y_ = cl::Buffer(
			context_,
			CL_MEM_READ_WRITE,
			sizeof(cl_float2) * data_size_,
			NULL,
			&err_);
	cl_buffer_acc_ = cl::Buffer(
			context_,
			CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
			sizeof(cl_float) * vec_out.size(),
			(void*)&vec_out[0],
			&err_);
	cl_buffer_short_ = cl::Buffer(
			context_,
			CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
			sizeof(short) * in_short.size(),
			(void*)&in_short[0],
			&err_);
	if (!pipeline_)
		queue_.finish();
	ptime after = microsec_clock::universal_time();
	return after - before;
}

time_duration cl_fft::run_fft(size_t sub_vec)
{
	ptime before = microsec_clock::universal_time();
	// compute fft
	double p = log2(sub_vec_size_);
	for (int i = 1; i <= ((sub_vec_size_) >> 1); i *= 2) {
		// enqueue the new parameter p
		err_ = kernel_fft_.setArg(2, i);
		// make the computation
		err_ = queue_.enqueueNDRangeKernel(
				kernel_fft_,
				cl::NullRange,
				cl::NDRange((sub_vec_size_) >> 1, sub_vec),
				cl::NullRange,
				NULL,
				&event_);
		if (!pipeline_)
			queue_.finish();
		if (i != ((sub_vec_size_) >> 1)) {
			err_ = queue_.enqueueCopyBuffer(
					cl_buffer_out_y_,
					cl_buffer_in_x_,
					0,
					0,
					data_size_ * sizeof(cl_float2),
					NULL,
					&event_);
			if (!pipeline_)
				queue_.finish();
		}
	}
	ptime after = microsec_clock::universal_time();
	return after - before;
}

time_duration cl_fft::run_prepare() {
	ptime before = microsec_clock::universal_time();
	err_ = queue_.enqueueNDRangeKernel(
			kernel_prepare_,
			cl::NullRange,
			cl::NDRange(data_size_),
			cl::NullRange,
			NULL,
			&event_);
	if (!pipeline_)
		queue_.finish();
	ptime after = microsec_clock::universal_time();
	return after - before;
}

time_duration cl_fft::run_acc(size_t sub_vec)
{
	ptime before = microsec_clock::universal_time();
	err_ = queue_.enqueueNDRangeKernel(
			kernel_acc_,
			cl::NullRange,
			cl::NDRange(sub_vec_size_, sub_vec),
			cl::NullRange,
			NULL,
			&event_);
	if (!pipeline_)
		queue_.finish();
	ptime after = microsec_clock::universal_time();
	return after - before;
}

time_duration cl_fft::gpu2cpu(std::vector<float>& out)
{
	ptime before = microsec_clock::universal_time();
	err_ = queue_.enqueueReadBuffer(
			cl_buffer_acc_,
			CL_TRUE,
			0,
			out.size() * sizeof(cl_float),
			&out[0],
			NULL,
			&event_);
	if (!pipeline_)
		queue_.finish();
	ptime after = microsec_clock::universal_time();
	return after - before;
}

time_duration cl_fft::run_multiple(
		std::vector<std::complex<float> >& in_out,
		size_t sub_vec)
{
	data_size_ = in_out.size();
	sub_vec_size_ = data_size_ / sub_vec;
	std::vector<float> vec_out;
	vec_out.assign(sub_vec_size_, 0.0f);
	// prepare and copy the memories
	ptime before;
	ptime after;
	kernel_fft_ = cl::Kernel(program_, "fftRadix2Kernel", &err_);
	kernel_acc_ = cl::Kernel(program_, "accumulate", &err_);
	kernel_prepare_ = cl::Kernel(program_, "prepare", &err_);
	std::vector<short> in;
	{
		std::vector<std::complex<float> >::const_iterator ite;
		for (ite = in_out.begin(); ite != in_out.end(); ++ite) {
			in.push_back((short)(ite->real()));
		}
	}
	before = microsec_clock::universal_time();
	time_duration cpu_2_gpu = cpu2gpu(in, vec_out);
	//set the arguments of our kernel_
	err_ = kernel_prepare_.setArg(0, cl_buffer_short_);
	err_ = kernel_prepare_.setArg(1, cl_buffer_in_x_);
	err_ = kernel_fft_.setArg(0, cl_buffer_in_x_);
	err_ = kernel_fft_.setArg(1, cl_buffer_out_y_);
	err_ = kernel_acc_.setArg(0, cl_buffer_out_y_);
	err_ = kernel_acc_.setArg(1, cl_buffer_acc_);
	//Wait for the command queue_ to finish these commands before proceeding
	if (pipeline_)
		queue_.finish();
	time_duration pre_time = run_prepare();
	time_duration fft_time = run_fft(sub_vec);
	time_duration acc_time = run_acc(sub_vec);
	time_duration gpu_2_cpu = gpu2cpu(vec_out);
	if (pipeline_)
		queue_.finish();
	after = microsec_clock::universal_time();
	in_out.resize(vec_out.size());
	for (size_t i = 0; i < vec_out.size(); ++i)
		in_out[i] = std::complex<float>(vec_out[i], 0.0f);
	time_duration total = after - before;
	std::cout << "CPU => GPU      : " << cpu_2_gpu << std::endl;
	std::cout << "prepare time    : " << pre_time << std::endl;
	std::cout << "fft time        : " << fft_time << std::endl;
	std::cout << "acc time        : " << acc_time << std::endl;
	std::cout << "GPU => CPU      : " << gpu_2_cpu << std::endl;
	return total;
}
