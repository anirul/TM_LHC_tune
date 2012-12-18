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

#include <algorithm>

#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "bunch_buffer.h"
#include "gsl_svd.h"

using namespace boost::posix_time;

bunch_buffer_f::bunch_buffer_f() : fft_instance_(NULL) {}

bunch_buffer_d::bunch_buffer_d() : fft_instance_(NULL) {}

bunch_buffer_f::bunch_buffer_f(const bunch_buffer_f& bb)
: fft_instance_(bb.fft_instance_)
{
	buffers_.insert(
			buffers_.end(),
			bb.buffers_.begin(),
			bb.buffers_.end());
	bunch_pattern_.insert(
			bunch_pattern_.end(),
			bb.bunch_pattern_.begin(),
			bb.bunch_pattern_.end());
}

bunch_buffer_d::bunch_buffer_d(const bunch_buffer_d& bb)
: fft_instance_(bb.fft_instance_)
{
	buffers_.insert(
			buffers_.end(),
			bb.buffers_.begin(),
			bb.buffers_.end());
	bunch_pattern_.insert(
			bunch_pattern_.end(),
			bb.bunch_pattern_.begin(),
			bb.bunch_pattern_.end());
}

bunch_buffer_f& bunch_buffer_f::operator=(const bunch_buffer_f& bb) {
	fft_instance_ = bb.fft_instance_;
	buffers_.clear();
	bunch_pattern_.clear();
	buffers_.insert(
			buffers_.end(),
			bb.buffers_.begin(),
			bb.buffers_.end());
	bunch_pattern_.insert(
			bunch_pattern_.end(),
			bb.bunch_pattern_.begin(),
			bb.bunch_pattern_.end());
	return *this;
}

bunch_buffer_d& bunch_buffer_d::operator=(const bunch_buffer_d& bb) {
	fft_instance_ = bb.fft_instance_;
	buffers_.clear();
	bunch_pattern_.clear();
	buffers_.insert(
			buffers_.end(),
			bb.buffers_.begin(),
			bb.buffers_.end());
	bunch_pattern_.insert(
			bunch_pattern_.end(),
			bb.bunch_pattern_.begin(),
			bb.bunch_pattern_.end());
	return *this;
}

bunch_buffer_f& bunch_buffer_f::operator+=(const bunch_buffer_f& bb) {
	if (!fft_instance_)
		fft_instance_ = bb.fft_instance_;
	buffers_.insert(
			buffers_.end(),
			bb.buffers_.begin(),
			bb.buffers_.end());
	bunch_pattern_.insert(
			bunch_pattern_.end(),
			bb.bunch_pattern_.begin(),
			bb.bunch_pattern_.end());
	return *this;
}

bunch_buffer_d& bunch_buffer_d::operator+=(const bunch_buffer_d& bb) {
	if (!fft_instance_)
		fft_instance_ = bb.fft_instance_;
	buffers_.insert(
			buffers_.end(),
			bb.buffers_.begin(),
			bb.buffers_.end());
	bunch_pattern_.insert(
			bunch_pattern_.end(),
			bb.bunch_pattern_.begin(),
			bb.bunch_pattern_.end());
	return *this;
}

bunch_buffer_f::bunch_buffer_f(
		const std::vector<short>& data,
		const std::vector<short>& bunch_pattern,
		i_fft_f* fft_instance)
: fft_instance_(fft_instance)
{
	bunch_pattern_ = bunch_pattern;
	for (unsigned long i = 0; i < bunch_pattern_.size(); ++i) {
		buffers_.push_back(
				acquisition_buffer_f(
						data,
						fft_instance,
						bunch_pattern_.size(),
						i));
	}
}

bunch_buffer_d::bunch_buffer_d(
		const std::vector<short>& data,
		const std::vector<short>& bunch_pattern,
		i_fft_d* fft_instance)
: fft_instance_(fft_instance)
{
	bunch_pattern_ = bunch_pattern;
	for (unsigned long i = 0; i < bunch_pattern.size(); ++i) {
		buffers_.push_back(
				acquisition_buffer_d(
						data,
						fft_instance,
						bunch_pattern.size(),
						i));
	}
}

bunch_buffer_f::~bunch_buffer_f() {}

bunch_buffer_d::~bunch_buffer_d() {}

bunch_buffer_f::bunch_buffer_f(
		const std::string& file_name,
		i_fft_f* fft_instance)
: fft_instance_(fft_instance)
{
	if (file_name.find(".gz") != std::string::npos) {
		load_gzip(file_name);
		return;
	}
	if (file_name.find(".xml") != std::string::npos) {
		load_txt(file_name);
		return;
	}
}

bunch_buffer_d::bunch_buffer_d(
		const std::string& file_name,
		i_fft_d* fft_instance)
: fft_instance_(fft_instance)
{
	if (file_name.find(".gz") != std::string::npos) {
		load_gzip(file_name);
		return;
	}
	if (file_name.find(".xml") != std::string::npos) {
		load_txt(file_name);
		return;
	}
}

std::vector<unsigned long> bunch_buffer_f::peak_detect(
		const unsigned long min,
		const unsigned long max)
{
	std::vector<unsigned long> ret;
	for (unsigned long i = 0; i < bunch_pattern_.size(); ++i)
		ret.push_back(buffers_[i].peak_detect(min, max));
	return ret;
}

std::vector<unsigned long> bunch_buffer_d::peak_detect(
		const unsigned long min,
		const unsigned long max)
{
	std::vector<unsigned long> ret;
	for (unsigned long i = 0; i < bunch_pattern_.size(); ++i)
		ret.push_back(buffers_[i].peak_detect(min, max));
	return ret;
}

std::vector<short> bunch_buffer_f::get_bunch_pattern() const {
	return bunch_pattern_;
}

std::vector<short> bunch_buffer_d::get_bunch_pattern() const {
	return bunch_pattern_;
}

time_duration bunch_buffer_f::normalize(std::vector<float>& inout) const {
	ptime before = microsec_clock::universal_time();
	float max = inout[0];
	for (unsigned int i = 0; i < inout.size(); ++i)
		if (inout[i] > max) max = inout[i];
	for (unsigned int i = 0; i < inout.size(); ++i)
		inout[i] = inout[i] / max;
	ptime after = microsec_clock::universal_time();
	return after - before;
}

time_duration bunch_buffer_d::normalize(std::vector<double>& inout) const {
	ptime before = microsec_clock::universal_time();
	double max = inout[0];
	for (unsigned int i = 0; i < inout.size(); ++i)
		if (inout[i] > max) max = inout[i];
	for (unsigned int i = 0; i < inout.size(); ++i)
		inout[i] = inout[i] / max;
	ptime after = microsec_clock::universal_time();
	return after - before;
}

void bunch_buffer_f::buffer(
		const unsigned long index,
		std::vector<float>& out) const
{
	if (index >= bunch_pattern_.size())
		throw std::runtime_error(
				"bunch_buffer_f::buffer := index out of band");
	buffers_[index].buffer_real(out);
}

void bunch_buffer_d::buffer(
		const unsigned long index,
		std::vector<double>& out) const
{
	if (index >= bunch_pattern_.size())
		throw std::runtime_error(
				"bunch_buffer_d::buffer := index out of band");
	buffers_[index].buffer_real(out);
}

size_t bunch_buffer_f::bunch_count() const {
	return bunch_pattern_.size();
}

size_t bunch_buffer_d::bunch_count() const {
	return bunch_pattern_.size();
}

size_t bunch_buffer_f::buffer_size() const {
	if (buffers_.size())
		return buffers_[0].size();
	return 0;
}

size_t bunch_buffer_d::buffer_size() const {
	if (buffers_.size())
		return buffers_[0].size();
	return 0;
}

time_duration bunch_buffer_f::notch() {
	ptime begin = microsec_clock::universal_time();
	for (unsigned long i = 0; i < bunch_pattern_.size(); ++i)
		buffers_[i].notch();
	ptime end = microsec_clock::universal_time();
	return end - begin;
}

time_duration bunch_buffer_d::notch() {
	ptime begin = microsec_clock::universal_time();
	for (unsigned long i = 0; i < bunch_pattern_.size(); ++i)
		buffers_[i].notch();
	ptime end = microsec_clock::universal_time();
	return end - begin;
}

time_duration bunch_buffer_f::average() {
	ptime before = microsec_clock::universal_time();
	for (unsigned long i = 0; i < bunch_pattern_.size(); ++i)
		buffers_[i].average();
	ptime after = microsec_clock::universal_time();
	return after - before;
}

time_duration bunch_buffer_d::average() {
	ptime before = microsec_clock::universal_time();
	for (unsigned long i = 0; i < bunch_pattern_.size(); ++i)
		buffers_[i].average();
	ptime after = microsec_clock::universal_time();
	return after - before;
}

time_duration bunch_buffer_f::accumulate(std::vector<float>& out) {
	ptime before = microsec_clock::universal_time();
	std::vector<float> temp;
	out.assign(buffer_size(), 0.0f);
	for (unsigned int i = 0; i < bunch_count(); ++i) {
		buffer(i, temp);
		if (!out.size())
			out.resize(temp.size());
		for (size_t i = 0; i < temp.size(); ++i)
			out[i] += temp[i];
	}
	ptime after = microsec_clock::universal_time();
	return after - before;
}

time_duration bunch_buffer_d::accumulate(std::vector<double>& out) {
	ptime before = microsec_clock::universal_time();
	std::vector<double> temp;
	out.assign(buffer_size(), 0.0f);
	for (unsigned int i = 0; i < bunch_count(); ++i) {
		buffer(i, temp);
		if (!out.size())
			out.resize(temp.size());
		for (size_t i = 0; i < temp.size(); ++i)
			out[i] += temp[i];
	}
	ptime after = microsec_clock::universal_time();
	return after - before;
}

float bunch_buffer_f::check_rms() {
	float acc = 0.0f;
	for (unsigned long i = 0; i < bunch_pattern_.size(); ++i)
		acc += buffers_[i].check_rms();
	return acc / (float)bunch_pattern_.size();
}

double bunch_buffer_d::check_rms() {
	double acc = 0.0;
	for (unsigned long i = 0; i < bunch_pattern_.size(); ++i)
		acc += buffers_[i].check_rms();
	return acc / (double)bunch_pattern_.size();
}

time_duration bunch_buffer_f::svd(float threshold) {
	ptime begin;
	ptime end;
	begin = microsec_clock::universal_time();
	// M x N
	gsl::matrix A(buffer_size(), bunch_count());
	for (int y = 0; y < bunch_count(); ++y) {
		for (int x = 0; x < buffer_size(); ++x) {
			A(x, y) = buffers_[y][x];
		}
	}
	// N x N
	gsl::matrix X(bunch_count(), bunch_count());
	gsl::vector work(bunch_count());
	gsl::vector S(bunch_count());
	gsl::matrix V(bunch_count(), bunch_count());
	gsl::matrix U = A;

	// compute SVD
	SVD_mod(U, X, V, S, work);

	// S -> s
	gsl::matrix s(bunch_count(), bunch_count());
	for (size_t i = 0; i < bunch_count(); ++i) {
		if (S[i] < threshold) {
			s(i, i) = 0.0f;
		} else {
			s(i, i) = S[i];
		}
	}

	// vt = V^T
	gsl::matrix vt = transpose(V);

	// out = U s vt (out ~ A)
	gsl::matrix out = U * s * vt;
	for (int y = 0; y < bunch_count(); ++y) {
		for (int x = 0; x < bunch_count(); ++x) {
			buffers_[y][x] = out(y, x);
		}
	}
	end = microsec_clock::universal_time();
	return end - begin;
}

time_duration bunch_buffer_d::svd(double threshold) {
	ptime begin;
	ptime end;
	begin = microsec_clock::universal_time();

	// M x N
	gsl::matrix A(buffer_size(), bunch_count());
	for (int y = 0; y < bunch_count(); ++y) {
		for (int x = 0; x < buffer_size(); ++x) {
			A(x, y) = buffers_[y][x];
		}
	}
	// N x N
	gsl::matrix X(bunch_count(), bunch_count());
	gsl::vector work(bunch_count());
	gsl::vector S(bunch_count());
	gsl::matrix V(bunch_count(), bunch_count());
	gsl::matrix U = A;

	// compute SVD
	SVD_mod(U, X, V, S, work);

	// S -> s
	gsl::matrix s(bunch_count(), bunch_count());
	for (size_t i = 0; i < bunch_count(); ++i) {
		if (S[i] < threshold) {
			s(i, i) = 0.0f;
		} else {
			s(i, i) = S[i];
		}
	}

	// vt = V^T
	gsl::matrix vt = transpose(V);

	// out = U s vt (out ~ A)
	gsl::matrix out = U * s * vt;
	for (int y = 0; y < bunch_count(); ++y) {
		for (int x = 0; x < bunch_count(); ++x) {
			buffers_[y][x] = out(y, x);
		}
	}
	end = microsec_clock::universal_time();
	return end - begin;
}

void bunch_buffer_f::clean(size_t begin, size_t end) {
	for (size_t i = 0; i < bunch_pattern_.size(); ++i)
		buffers_[i].clean(begin, end);
}

void bunch_buffer_d::clean(size_t begin, size_t end) {
	for (size_t i = 0; i < bunch_pattern_.size(); ++i)
		buffers_[i].clean(begin, end);
}

time_duration bunch_buffer_f::fft_single() {
	time_duration total = minutes(0);
	for (unsigned long i = 0; i < bunch_pattern_.size(); ++i) {
		total += fft_instance_->run_single(
				buffers_[i].complex_buffer_);
	}
	return total;
}

time_duration bunch_buffer_d::fft_single() {
	time_duration total = minutes(0);
	for (unsigned long i = 0; i < bunch_pattern_.size(); ++i) {
		total += fft_instance_->run_single(
				buffers_[i].complex_buffer_);
	}
	return total;
}

time_duration bunch_buffer_f::fft_multiple() {
	std::vector<std::complex<float> > total;
	for (unsigned long i = 0; i < bunch_pattern_.size(); ++i)
		total.insert(
				total.end(),
				buffers_[i].complex_buffer_.begin(),
				buffers_[i].complex_buffer_.end());
	time_duration duration =
			fft_instance_->run_multiple(total, bunch_pattern_.size());
	for (unsigned long i = 0; i < bunch_pattern_.size(); ++i) {
		size_t half = buffers_[i].complex_buffer_.size() / 2;
		buffers_[i].complex_buffer_.resize(half);
		memcpy(
				&(buffers_[i].complex_buffer_[0]),
				&total[i * half * 2],
				half * sizeof(std::complex<float>));
	}
	return duration;
}

time_duration bunch_buffer_d::fft_multiple() {
	std::vector<std::complex<double> > total;
	for (unsigned long i = 0; i < bunch_pattern_.size(); ++i)
		total.insert(
				total.end(),
				buffers_[i].complex_buffer_.begin(),
				buffers_[i].complex_buffer_.end());
	time_duration duration =
			fft_instance_->run_multiple(total, bunch_pattern_.size());
	for (unsigned long i = 0; i < bunch_pattern_.size(); ++i) {
		size_t half = buffers_[i].complex_buffer_.size() / 2;
		buffers_[i].complex_buffer_.resize(half);
		memcpy(
				&(buffers_[i].complex_buffer_[0]),
				&total[i * half * 2],
				half * sizeof(std::complex<double>));
	}
	return duration;
}

time_duration bunch_buffer_f::amplitude() {
	ptime before = microsec_clock::universal_time();
	for (unsigned long i = 0; i < bunch_pattern_.size(); ++i)
		buffers_[i].amplitude();
	ptime after = microsec_clock::universal_time();
	return after - before;
}

time_duration bunch_buffer_d::amplitude() {
	ptime before = microsec_clock::universal_time();
	for (unsigned long i = 0; i < bunch_pattern_.size(); ++i)
		buffers_[i].amplitude();
	ptime after = microsec_clock::universal_time();
	return after - before;
}

void bunch_buffer_f::phase_deg() {
	for (unsigned long i = 0; i < bunch_pattern_.size(); ++i)
		buffers_[i].phase_deg();
}

void bunch_buffer_d::phase_deg() {
	for (unsigned long i = 0; i < bunch_pattern_.size(); ++i)
		buffers_[i].phase_deg();
}

void bunch_buffer_f::log10() {
	for (unsigned long i = 0; i < bunch_pattern_.size(); ++i)
		buffers_[i].log10();
}

void bunch_buffer_d::log10() {
	for (unsigned long i = 0; i < bunch_pattern_.size(); ++i)
		buffers_[i].log10();
}

void bunch_buffer_f::save_txt(
		std::ostream& os,
		const std::string& time_stamp)
{
	os
	<< "<bunch_buffer_f time-stamp=\"" << time_stamp
	<< "\" bunch-pattern=\"";
	for (size_t i = 0; i < bunch_pattern_.size(); ++i)
		os << bunch_pattern_[i] << " ";
	os
	<< "\">" << std::endl;
	for (unsigned long i = 0; i < bunch_pattern_.size(); ++i) {
		os << "\t<buffer" << i << ">" << std::endl;
		os << "\t\t";
		buffers_[i].save_txt(os);
		os << std::endl;
		os << "\t</buffer" << i << ">" << std::endl;
	}
	os
	<< "</bunch_buffer_f>" << std::endl;
}

void bunch_buffer_d::save_txt(
		std::ostream& os,
		const std::string& time_stamp)
{
	os
	<< "<bunch_buffer_d time-stamp=\"" << time_stamp
	<< "\" bunch-pattern=\"";
	for (size_t i = 0; i < bunch_pattern_.size(); ++i)
		os << bunch_pattern_[i] << " ";
	os
	<< "\">" << std::endl;
	for (unsigned long i = 0; i < bunch_pattern_.size(); ++i) {
		os << "\t<buffer" << i << ">" << std::endl;
		os << "\t\t";
		buffers_[i].save_txt(os);
		os << std::endl;
		os << "\t</buffer" << i << ">" << std::endl;
	}
	os
	<< "</bunch_buffer_d>" << std::endl;
}

bool bunch_buffer_f::save_txt(
		const std::string& file_name,
		const std::string& time_stamp)
{
	std::ofstream ofs(file_name.c_str());
	if (!ofs.is_open()) return false;
	save_txt(ofs, time_stamp);
	ofs.close();
	return true;
}

bool bunch_buffer_d::save_txt(
		const std::string& file_name,
		const std::string& time_stamp)
{
	std::ofstream ofs(file_name.c_str());
	if (!ofs.is_open()) return false;
	save_txt(ofs, time_stamp);
	ofs.close();
	return true;
}

bool bunch_buffer_f::load_txt(const std::string& file_name) {
	std::ifstream ifs(file_name.c_str());
	if (!ifs.is_open()) return false;
	load_txt(ifs);
	ifs.close();
	return true;
}

bool bunch_buffer_d::load_txt(const std::string& file_name) {
	std::ifstream ifs(file_name.c_str());
	if (!ifs.is_open()) return false;
	load_txt(ifs);
	ifs.close();
	return true;
}

void bunch_buffer_f::load_txt(std::istream& is) {
	bunch_pattern_.clear();
	while (!is.eof()) {
		std::string item;
		std::getline(is, item);
		if (item.find("bunch-pattern=") != std::string::npos) {
			size_t pos = item.find("bunch-pattern=");
			std::string sub_item = item.substr(
					item.find_first_of("\"", pos) + 1,
					item.find_last_of("\"") - item.find_first_of("\"", pos) - 2);
			std::stringstream ss(sub_item);
			short token = 0;
			while (!ss.eof()) {
				ss >> token;
				bunch_pattern_.push_back(token);
			}
		}
		if (item.find("<buffer") != std::string::npos) {
			std::string values;
			std::getline(is, values);
			buffers_.push_back(acquisition_buffer_f(values, fft_instance_));
		}
	}
}

void bunch_buffer_d::load_txt(std::istream& is) {
	bunch_pattern_.clear();
	while (!is.eof()) {
		std::string item;
		std::getline(is, item);
		if (item.find("bunch-pattern=") != std::string::npos) {
			size_t pos = item.find("bunch-pattern=");
			std::string sub_item = item.substr(
					item.find_first_of("\"", pos) + 1,
					item.find_last_of("\"") - item.find_first_of("\"", pos) - 2);
			std::stringstream ss(sub_item);
			short token = 0;
			while (!ss.eof()) {
				ss >> token;
				bunch_pattern_.push_back(token);
			}
		}
		if (item.find("<buffer") != std::string::npos) {
			std::string values;
			std::getline(is, values);
			buffers_.push_back(acquisition_buffer_d(values, fft_instance_));
		}
	}
}

bool bunch_buffer_f::save_gzip(
		const std::string& file_name,
		const std::string& time_stamp)
{
	std::ofstream ofs(file_name.c_str(), std::ofstream::binary);
	if (!ofs.is_open()) return false;
	boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
	in.push(boost::iostreams::gzip_compressor());
	std::stringstream data;
	save_txt(data, time_stamp);
	in.push(data);
	boost::iostreams::copy(in, ofs);
	ofs.close();
	return true;
}

bool bunch_buffer_d::save_gzip(
		const std::string& file_name,
		const std::string& time_stamp)
{
	std::ofstream ofs(file_name.c_str(), std::ofstream::binary);
	if (!ofs.is_open()) return false;
	boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
	in.push(boost::iostreams::gzip_compressor());
	std::stringstream data;
	save_txt(data, time_stamp);
	in.push(data);
	boost::iostreams::copy(in, ofs);
	ofs.close();
	return true;
}

bool bunch_buffer_f::load_gzip(const std::string& file_name) {
	std::stringstream ss("");
	{
		std::ifstream ifs(
				file_name.c_str(),
				std::ios_base::in | std::ios_base::binary);
		if (!ifs.is_open()) return false;
		boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
		in.push(boost::iostreams::gzip_decompressor());
		in.push(ifs);
		boost::iostreams::copy(in, ss);
		ifs.close();
	}
	ss.seekg(0, std::ios::beg);
	load_txt(ss);
	return true;
}

bool bunch_buffer_d::load_gzip(const std::string& file_name) {
	std::stringstream ss("");
	{
		std::ifstream ifs(
				file_name.c_str(),
				std::ios_base::in | std::ios_base::binary);
		if (!ifs.is_open()) return false;
		boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
		in.push(boost::iostreams::gzip_decompressor());
		in.push(ifs);
		boost::iostreams::copy(in, ss);
		ifs.close();
	}
	ss.seekg(0, std::ios::beg);
	load_txt(ss);
	return true;
}

bool bunch_buffer_f::empty() const {
	for (int i = 0; i < buffers_.size(); ++i) {
		if (!buffers_[i].empty()) {
			return false;
		}
	}
	return true;
}

bool bunch_buffer_d::empty() const {
	for (int i = 0; i < buffers_.size(); ++i) {
		if (!buffers_[i].empty()) {
			return false;
		}
	}
	return true;
}

void bunch_buffer_f::clear() {
	buffers_.clear();
	bunch_pattern_.clear();
}

void bunch_buffer_d::clear() {
	buffers_.clear();
	bunch_pattern_.clear();
}

void bunch_buffer_f::resize(size_t size) {
	for (size_t i = 0; i < buffers_.size(); ++i)
		buffers_[i].resize(size);
}

void bunch_buffer_d::resize(size_t size) {
	for (size_t i = 0; i < buffers_.size(); ++i)
		buffers_[i].resize(size);
}

