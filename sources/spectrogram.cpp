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

#include <stdexcept>
#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "bunch_buffer.h"
#include "spectrogram.h"
#include "bunch_buffer.h"

namespace fs = boost::filesystem;
namespace bt = boost::posix_time;

void spectrogram::accumulate(
	std::vector<float>& out,
	const std::vector<float>& in1,
	const std::vector<float>& in2) const
{
	unsigned int min = (in1.size() < in2.size()) ? in1.size() : in2.size();
	out.resize(min);
	for (unsigned int i = 0; i < out.size(); ++i)
		out[i] = in1[i] + in2[i];
}

void spectrogram::divide(
	std::vector<float>& inout,
	float divider) const
{
	for (unsigned int i = 0; i < inout.size(); ++i)
		inout[i] = inout[i] / divider;
}

void spectrogram::normalize(std::vector<float>& inout) const {
	float max = inout[0];
	for (unsigned int i = 0; i < inout.size(); ++i)
		if (inout[i] > max) max = inout[i];
	for (unsigned int i = 0; i < inout.size(); ++i)
		inout[i] = inout[i] / max;
}

void spectrogram::average(
	const bunch_buffer_f& buffers,
	std::vector<float>& out) const 
{
	std::vector<float> temp;
	out.assign(buffers.buffer_size(), 0.0f);
	for (unsigned int i = 0; i < buffers.bunch_count(); ++i) {
		if (bunch_mask_[i]) {
			buffers.buffer(i, temp);
			accumulate(out, out, temp);
		}
	}
}

bunch_buffer_f spectrogram::buffer_from_file(
	const boost::filesystem::path& path) const 
{
	std::string full_path = fs::canonical(path).string();
	bunch_buffer_f bb(full_path);
	if (!bb.buffer_size()) {
		std::cout << std::endl;
		throw std::runtime_error("Error invalid file : " + full_path);
	}
	if (!bb.get_bunch_pattern().size()) {
		std::cout << std::endl;
		throw std::runtime_error("Invalid bunch pattern (empty)");
	}
	return bb;
}

long long spectrogram::time_stamp_from_file(
	const boost::filesystem::path& path) const 
{
	std::string full_path = fs::canonical(path).string();
	long long time_stamp = 0;
	std::string str_time_stamp = "";
	{ // get the time from file name
		str_time_stamp = full_path.substr(
			full_path.find_last_of("-") + 1,
			full_path.find_first_of(".") - 
			full_path.find_last_of("-") - 1);
		std::stringstream ss(str_time_stamp);
		ss >> time_stamp;
	}
	return time_stamp;
}

boost::posix_time::ptime spectrogram::ptime_from_file(
	const boost::filesystem::path& path) const
{
	std::string full_path = fs::canonical(path).string();
	long long time_stamp = time_stamp_from_file(path);
	boost::posix_time::ptime file_time;
	{ // convert to time
		file_time = boost::posix_time::from_time_t(
			time_stamp / 1000000000L);
		file_time += boost::posix_time::microseconds(
			(time_stamp % 1000000000L) / 1000);
	}
	return file_time;
}

spectrogram::spectrogram(uint32_t nb_acc, const std::bitset<16>& bunch_mask) 
	:	pitch_(0),
		nb_acc_(nb_acc),
		bunch_mask_(bunch_mask)
{
	bunch_pattern_.clear();
	data_.clear();
	time_.clear();
}

spectrogram::~spectrogram() {}

void spectrogram::load_files(
	const std::string& path, 
	const commands& cmd,
	int64_t start_time,
	int64_t end_time) 
{
	data_.clear();
	time_.clear();
	bunch_pattern_.clear();
	try {
		fs::path open_path(path.c_str());
		std::vector<fs::path> list_file;
		if (fs::exists(open_path) && fs::is_directory(open_path)) {
			std::copy(
				fs::directory_iterator(open_path), 
				fs::directory_iterator(), 
				std::back_inserter(list_file));
			sort(list_file.begin(), list_file.end());
		} else {
			throw std::runtime_error(path + " is not a directory!");
		}
		std::vector<fs::path>::iterator ite;
		bunch_buffer_f acc_bb;
		std::vector<float> temp;
		unsigned int acc_count = 0;
		unsigned int file_count = 0;
		for (ite = list_file.begin(); ite != list_file.end(); ++ite) {
			std::string full_path = fs::canonical(*ite).string();
			long long time_stamp = time_stamp_from_file(*ite);
			// check boundaries
			if (time_stamp < start_time || time_stamp > end_time)
				continue;
			boost::posix_time::ptime file_time = ptime_from_file(*ite);
			std::cout 
				<< "\rloading (data)  : " 
				<< ++file_count << "/" << list_file.size() << " "
				<< file_time << " "
				<< full_path;
			std::cout.flush();
			bunch_buffer_f bb = buffer_from_file(*ite);
			if (bb.empty()) {
				std::cout 
					<< std::endl
					<< "empty (data)    : " << full_path << " Not saved!" 
					<< std::endl;
				continue;
			}
			if (!pitch_) {
				pitch_ = bb.buffer_size() / 2;
			} 
			// save the bunch pattern
			if (bb.get_bunch_pattern() != bunch_pattern_) { 
				bunch_pattern_ = bb.get_bunch_pattern();
				std::cout << std::endl;
				std::cout << "bunch pattern   : ";
				for (size_t i = 0; i < bunch_pattern_.size(); ++i)
					std::cout << bunch_pattern_[i] << " ";
				std::cout << std::endl;
			}
			time_.push_back(time_stamp);
			acc_bb += bb;
			if (!(acc_count % nb_acc_)) {
				// here come the computing
				cmd(acc_bb);
				// apply bunch mask!
				average(acc_bb, temp);
				temp.resize(pitch_);
				normalize(temp);
				data_.insert(data_.end(), temp.begin(), temp.end());
				acc_bb.clear();
			}
			acc_count++;
		}
//		if (!data_.empty())
//			normalize(data_);
		std::cout << std::endl;
	} catch (const fs::filesystem_error& er) {
		std::cerr << "exception (fs)  : " << er.what() << std::endl;
	}
}

void spectrogram::save_dump(const std::string& file) const {
	FILE* fp = NULL;
	fp = fopen(file.c_str(), "wb");
	if (!fp)
		throw std::runtime_error("could not write file " + file);
	uint32_t version = 3;
	fwrite(&version, sizeof(uint32_t), 1, fp);
	fwrite(&pitch_, sizeof(uint32_t), 1, fp);
	uint32_t ulong = bunch_mask_.to_ulong();
	fwrite(&ulong, sizeof(uint32_t), 1, fp);
	fwrite(&nb_acc_, sizeof(uint32_t), 1, fp);
	{ // save bunch pattern
		size_t size = bunch_pattern_.size();
		fwrite(&size, sizeof(size_t), 1, fp);
		fwrite(&bunch_pattern_[0], sizeof(short), bunch_pattern_.size(), fp);
	}
	{ // save time
		size_t size = time_.size();
		fwrite(&size, sizeof(size_t), 1, fp);
		fwrite(&time_[0], sizeof(long long), time_.size(), fp);
	}
	{ // save data
		size_t size = data_.size();
		fwrite(&size, sizeof(size_t), 1, fp);
		fwrite(&data_[0], sizeof(float), data_.size(), fp);
	}
	fclose(fp);
}

void spectrogram::load_dump(const std::string& file) {
	FILE* fp = NULL;
	fp = fopen(file.c_str(), "rb");
	if (!fp)
		throw std::runtime_error("could not read file " + file);
	uint32_t version = 1;
	fread(&version, sizeof(uint32_t), 1, fp);
	fread(&pitch_, sizeof(uint32_t), 1, fp);
	{ // get the bunch mask
		uint32_t ulong = 0;
		fread(&ulong, sizeof(uint32_t), 1, fp);
		bunch_mask_ = std::bitset<16>(ulong);
	}
	fread(&nb_acc_, sizeof(uint32_t), 1, fp);
	if (version >= 3) { // load bunch pattern (if v3)
		size_t size = 0;
		fread(&size, sizeof(size_t), 1, fp);
		bunch_pattern_.resize(size);
		fread(&bunch_pattern_[0], sizeof(short), size, fp);
	} else {
		for (int i = 0; i < bunch_mask_.count(); ++i)
			bunch_pattern_.push_back(0);
	}
	if (version >= 2) { // load time (if v2)
		size_t size = 0;
		fread(&size, sizeof(size_t), 1, fp);
		time_.resize(size);
		fread(&time_[0], sizeof(long long), size, fp);
	}
	{ // load data
		size_t size = 0;
		fread(&size, sizeof(size_t), 1, fp);
		data_.resize(size);
		fread(&data_[0], sizeof(float), size, fp);
	}
	fclose(fp);
}

uint32_t spectrogram::pitch() const {
	return pitch_;
}

uint32_t spectrogram::line_count() const {
	if (!pitch_)
		throw std::runtime_error("invalid pitch");
	return data_.size() / pitch_;
}

const std::vector<float>& spectrogram::data() const {
	return data_;
}

const std::vector<short>& spectrogram::bunch_pattern() const {
	return bunch_pattern_;
}

const std::bitset<16>& spectrogram::bunch_mask() const {
	return bunch_mask_;
}

const float* spectrogram::line(uint32_t index, uint32_t nb_lines) const {
	uint32_t total_lines = data_.size() / pitch_;
	if (nb_lines > total_lines) {
      std::stringstream ss("");
      ss << "too many lines asked not that many available " << nb_lines << " > " << total_lines;
		throw std::runtime_error(ss.str());
   	}
	if (index + nb_lines > total_lines) 
		index = total_lines - nb_lines;
	return &data_[index * pitch_];
}

long long spectrogram::time(uint32_t index) const {
	if ((index * nb_acc_) < time_.size()) {
		return time_[index * nb_acc_];
	}
	return 0;
}
