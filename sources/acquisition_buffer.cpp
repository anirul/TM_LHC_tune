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

#include "acquisition_buffer.h"
#include <vector>
#include <iostream>
#include <limits>
#include <algorithm>

float rms_f(const std::vector<float>& vec) {
	float acc = 0.0f;
	for (unsigned long i = 0; i < vec.size(); ++i)
		acc += (vec[i] * vec[i]);
	acc /= (float)vec.size();
	// avoid start(0)
	if (acc == 0.0f)
		acc = std::numeric_limits<float>::epsilon();
	return sqrtf(acc);
}

double rms_d(const std::vector<double>& vec) {
	double acc = 0.0;
	for (unsigned long i = 0; i < vec.size(); ++i)
		acc += (vec[i] * vec[i]);
	acc /= (double)vec.size();
	// avoid sqrt(0)
	if (acc == 0.0)
		acc = std::numeric_limits<float>::epsilon();
	return sqrt(acc);
}

float average_f(const std::vector<float>& vec) {
	float acc = 0.0f;
	for (unsigned long i = 0; i < vec.size(); ++i)
		acc += vec[i];
	return acc / (float)vec.size();
}

float average_f(const std::vector<unsigned long>& vec) {
	unsigned long acc = 0;
	for (unsigned long i = 0; i < vec.size(); ++i)
		acc += vec[i];
	return (float)acc / (float)vec.size();
}

double average_d(const std::vector<double>& vec) {
	double acc = 0.0;
	for (unsigned long i = 0; i < vec.size(); ++i)
		acc += vec[i];
	return acc / (double)vec.size();
}

double average_d(const std::vector<unsigned long>& vec) {
	unsigned long acc = 0;
	for (unsigned long i = 0; i < vec.size(); ++i)
		acc += vec[i];
	return (double)acc / (double)vec.size();
}

acquisition_buffer_f::acquisition_buffer_f(
		const std::vector<short>& in,
		i_fft_f* fft_instance,
		unsigned long pitch,
		unsigned long offset)
			: fft_instance_(fft_instance)
{
	if (offset >= pitch)
		throw std::runtime_error("offset cannot be >= to pitch");
	complex_buffer_.resize(in.size());
	unsigned long j = 0;
	for (unsigned long i = offset; i < in.size(); i += pitch)
		complex_buffer_[j++] = std::complex<float>((float)in[i], 0.0f);
	complex_buffer_.resize(j);
}

acquisition_buffer_d::acquisition_buffer_d(
		const std::vector<short>& in,
		i_fft_d* fft_instance,
		unsigned long pitch,
		unsigned long offset)
			: fft_instance_(fft_instance)
{
	if (offset >= pitch)
		throw std::runtime_error("offset cannot be >= to pitch");
	complex_buffer_.resize(in.size());
	unsigned long j = 0;
	for (unsigned long i = offset; i < in.size(); i += pitch)
		complex_buffer_[j++] = std::complex<double>((double)in[i], 0.0);
	complex_buffer_.resize(j);
}

acquisition_buffer_f::acquisition_buffer_f(
		const std::string& values,
		i_fft_f* fft_instance)
			: fft_instance_(fft_instance)
{
	std::stringstream ss(values);
	load_txt(ss);
}

acquisition_buffer_d::acquisition_buffer_d(
		const std::string& values,
		i_fft_d* fft_instance)
			: fft_instance_(fft_instance)
{
	std::stringstream ss(values);
	load_txt(ss);
}

size_t acquisition_buffer_f::size() const {
	return complex_buffer_.size();
}

size_t acquisition_buffer_d::size() const {
	return complex_buffer_.size();
}

void acquisition_buffer_f::clean(size_t begin, size_t end) {
	for (size_t i = begin; i < end; ++i)
		complex_buffer_[i] = std::complex<float>(0.0f, 0.0f);
}

void acquisition_buffer_d::clean(size_t begin, size_t end) {
	for (size_t i = begin; i < end; ++i)
		complex_buffer_[i] = std::complex<double>(0.0, 0.0);
}

void acquisition_buffer_f::buffer_real(std::vector<float>& out) const {
	out.resize(complex_buffer_.size());
	for (unsigned i = 0; i < complex_buffer_.size(); ++i)
		out[i] = complex_buffer_[i].real();
}

void acquisition_buffer_d::buffer_real(std::vector<double>& out) const {
	out.resize(complex_buffer_.size());
	for (unsigned i = 0; i < complex_buffer_.size(); ++i)
		out[i] = complex_buffer_[i].real();
}

void acquisition_buffer_f::buffer_complex(std::vector<std::complex<float> >& out) const {
	out.resize(complex_buffer_.size());
	out = complex_buffer_;
}

void acquisition_buffer_d::buffer_complex(std::vector<std::complex<double> >& out) const {
	out.resize(complex_buffer_.size());
	out = complex_buffer_;
}

void acquisition_buffer_f::notch() {
	if (!complex_buffer_.size())
		throw std::runtime_error("notch empty complex buffer!");
	for (unsigned long i = 0; i < (complex_buffer_.size() - 1); ++i)
		complex_buffer_[i] -= complex_buffer_[i + 1];
	complex_buffer_.resize(complex_buffer_.size() - 1);
}

void acquisition_buffer_f::average() {
	float avg = 0.0f;
	for (size_t i = 0; i < complex_buffer_.size(); ++i)
		avg += complex_buffer_[i].real();
	avg /= complex_buffer_.size();
	for (size_t i = 0; i < complex_buffer_.size(); ++i)
		complex_buffer_[i].real() -= avg;
}

void acquisition_buffer_d::average() {
	double avg = 0.0f;
	for (size_t i = 0; i < complex_buffer_.size(); ++i)
		avg += complex_buffer_[i].real();
	avg /= complex_buffer_.size();
	for (size_t i = 0; i < complex_buffer_.size(); ++i)
		complex_buffer_[i].real() -= avg;
}

void acquisition_buffer_d::notch() {
	if (!complex_buffer_.size())
		throw std::runtime_error("notch empty complex buffer!");
	for (unsigned long i = 0; i < (complex_buffer_.size() - 1); ++i)
		complex_buffer_[i] -= complex_buffer_[i + 1];
	complex_buffer_.resize(complex_buffer_.size() - 1);
}

float acquisition_buffer_f::check_rms() {
	size_t ten = complex_buffer_.size() / 10;
	std::vector<float> real;
	buffer_real(real);
	std::vector<float> begin(real.begin(), real.begin() + ten);
	std::vector<float> end(real.end() - ten, real.end());
	return rms_f(begin) / rms_f(end);
}

double acquisition_buffer_d::check_rms() {
	size_t ten = complex_buffer_.size() / 10;
	std::vector<double> real;
	buffer_real(real);
	std::vector<double> begin(real.begin(), real.begin() + ten);
	std::vector<double> end(real.end() - ten, real.end());
	return rms_d(begin) / rms_d(end);
}

void acquisition_buffer_f::fft() {
	if (!complex_buffer_.size())
		throw std::runtime_error("empty complex buffer!");
	fft_instance_->prepare(complex_buffer_);
	fft_instance_->run(complex_buffer_);
	// only the bottom half is interesting
	complex_buffer_.resize(complex_buffer_.size() / 2);
}

void acquisition_buffer_d::fft() {
	if (!complex_buffer_.size())
		throw std::runtime_error("empty complex buffer!");
	fft_instance_->prepare(complex_buffer_);
	fft_instance_->run(complex_buffer_);
	// only the bottom half is interesting
	complex_buffer_.resize(complex_buffer_.size() / 2);
}

void acquisition_buffer_f::amplitude() {
	for (unsigned long i = 0; i < complex_buffer_.size(); ++i)
		complex_buffer_[i] = std::complex<float>(
				sqrtf(std::norm(complex_buffer_[i])),
				0.0f);
}

void acquisition_buffer_d::amplitude() {
	for (unsigned long i = 0; i < complex_buffer_.size(); ++i)
		complex_buffer_[i] = std::complex<double>(
				sqrt(std::norm(complex_buffer_[i])),
				0.0);
}

void acquisition_buffer_f::phase_deg() {
	const float rad_2_deg = 180.0f / (float)M_PI;
	for (unsigned long i = 0; i < complex_buffer_.size(); ++i)
		complex_buffer_[i] = std::complex<float>(
				std::arg(complex_buffer_[i]) * rad_2_deg,
				0.0f);
}

void acquisition_buffer_d::phase_deg() {
	const double rad_2_deg = 180.0 / M_PI;
	for (unsigned long i = 0; i < complex_buffer_.size(); ++i)
		complex_buffer_[i] = std::complex<double>(
				std::arg(complex_buffer_[i]) * rad_2_deg,
				0.0);
}

void acquisition_buffer_f::log10() {
	for (unsigned long i = 0; i < complex_buffer_.size(); ++i)
		complex_buffer_[i] = std::complex<float>(
				::log10f(complex_buffer_[i].real()),
				 0.0f);
}

void acquisition_buffer_d::log10() {
	for (unsigned long i = 0; i < complex_buffer_.size(); ++i)
		complex_buffer_[i] = std::complex<double>(
				::log10(complex_buffer_[i].real()),
				 0.0);
}

unsigned long acquisition_buffer_f::peak_detect(
		unsigned long min,
		unsigned long max)
{
	if (max >= complex_buffer_.size())
		throw std::runtime_error("peak_detect<float> : max >= size");
	if (min > max)
		throw std::runtime_error("peak_detect<float> : min > max");
	float peak_value = std::numeric_limits<float>::min();
	unsigned int peak_index = min;
	for (unsigned long i = min; i < max; ++i) {
		if (complex_buffer_[i].real() > peak_value) {
			peak_value = complex_buffer_[i].real();
			peak_index = i;
		}
	}
	return peak_index;
}

unsigned long acquisition_buffer_d::peak_detect(
		unsigned long min,
		unsigned long max)
{
	if (max >= complex_buffer_.size())
		throw std::runtime_error("peak_detect<double> : max >= size");
	if (min > max)
		throw std::runtime_error("peak_detect<double> : min > max");
	double peak_value = std::numeric_limits<double>::min();
	unsigned int peak_index = min;
	for (unsigned long i = min; i <= max; ++i) {
		if (complex_buffer_[i].real() > peak_value) {
			peak_value = complex_buffer_[i].real();
			peak_index = i;
		}
	}
	return peak_index;
}

void acquisition_buffer_f::save_txt(std::ostream& os) {
	for (unsigned long i = 0; i < complex_buffer_.size(); ++i)
		os << complex_buffer_[i].real() << ", ";
}

void acquisition_buffer_d::save_txt(std::ostream& os) {
	for (unsigned long i = 0; i < complex_buffer_.size(); ++i)
		os << complex_buffer_[i].real() << ", ";
}

void acquisition_buffer_f::load_txt(std::istream& is) {
	while (!is.eof()) {
		std::string item;
		is >> item;
		if (item != std::string(",") && item !=  std::string("")) {
			std::stringstream ss(item);
			float f = 0.0f;
			ss >> f;
			complex_buffer_.push_back(std::complex<float>(f, 0.0f));
		}
	}
}

void acquisition_buffer_d::load_txt(std::istream& is) {
	while (!is.eof()) {
		std::string item;
		is >> item;
		if (item != std::string(",") && item !=  std::string("")) {
			std::stringstream ss(item);
			float d = 0.0f;
			ss >> d;
			complex_buffer_.push_back(std::complex<double>(d, 0.0));
		}
	}
}

bool acquisition_buffer_f::empty() const {
	for (size_t i = 0; i < complex_buffer_.size(); ++i)
		if (complex_buffer_[i].real() != 0.0f)
			return false;
	return true;
}

bool acquisition_buffer_d::empty() const {
	for (size_t i = 0; i < complex_buffer_.size(); ++i)
		if (complex_buffer_[i].real() != 0.0)
			return false;
	return true;
}

float& acquisition_buffer_f::operator[](size_t index) {
	return complex_buffer_[index].real();
}

const float& acquisition_buffer_f::operator[](size_t index) const {
	return complex_buffer_[index].real();
}

double& acquisition_buffer_d::operator[](size_t index ) {
	return complex_buffer_[index].real();
}

const double& acquisition_buffer_d::operator[](size_t index) const {
	return complex_buffer_[index].real();
}

void acquisition_buffer_f::resize(size_t size) {
	complex_buffer_.resize(size);
}

void acquisition_buffer_d::resize(size_t size) {
	complex_buffer_.resize(size);
}
