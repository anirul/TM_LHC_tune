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
	const std::vector<float>& in2)
{
	unsigned int min = (in1.size() < in2.size()) ? in1.size() : in2.size();
	out.resize(min);
	for (unsigned int i = 0; i < out.size(); ++i)
		out[i] = in1[i] + in2[i];
}

void spectrogram::divide(
	std::vector<float>& inout,
	float divider)
{
	for (unsigned int i = 0; i < inout.size(); ++i)
		inout[i] = inout[i] / divider;
}

void spectrogram::normalize(std::vector<float>& inout) {
	float max = inout[0];
	for (unsigned int i = 0; i < inout.size(); ++i)
		if (inout[i] > max) max = inout[i];
	for (unsigned int i = 0; i < inout.size(); ++i)
		inout[i] = inout[i] / max;
}

void spectrogram::average(
	const bunch_buffer_f& buffers,
	std::vector<float>& out,
	uint32_t bunch_mask)
{
	std::vector<float> temp;
	out.assign(buffers.buffer_size(), 0.0f);
	for (unsigned int i = 0; i < buffers.bunch_count(); ++i) {
		if ((0x1 << i) & (unsigned int)bunch_mask) {
			buffers.buffer(i, temp);
			accumulate(out, out, temp);
		}
	}
	normalize(out);
}

spectrogram::spectrogram(uint32_t nb_acc, uint32_t bunch_mask) 
	:	pitch_(0),
		nb_acc_(nb_acc),
		bunch_mask_(bunch_mask)
{
	data_.clear();
}

spectrogram::~spectrogram() {}

void spectrogram::load_files(const std::string& path, bool pre_notch) {
	data_.clear();
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
		std::vector<float> acc_vec;
		std::vector<float> temp;
		unsigned int acc_count = 0;
		unsigned int file_count = 0;
		for (ite = list_file.begin(); ite != list_file.end(); ++ite) {
			std::string full_path = fs::canonical(*ite).string();
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
			boost::posix_time::ptime file_time;
			{ // convert to time
				file_time = boost::posix_time::from_time_t(
					time_stamp / 1000000000L);
				file_time += boost::posix_time::microseconds(
					(time_stamp % 1000000000L) / 1000);
			}
			std::cout << "\rloading (data)  : " 
				<< ++file_count << "/" << list_file.size() << " "
				<< file_time << " "
				<< full_path;
			std::cout.flush();
			bunch_buffer_f bb(full_path);
			if (!pitch_) {
				pitch_ = bb.buffer_size() / 2;
			} 
			if (!bb.buffer_size())
				throw std::runtime_error("Error invalid file : " + full_path);
			if (!pre_notch) bb.notch();
			bb.fft();
			bb.amplitude();
			average(bb, temp, bunch_mask_);
			time_.insert(time_stamp);
			if (ite == list_file.begin()) 
				acc_vec.assign(bb.buffer_size(), 0.0f);
			accumulate(acc_vec, acc_vec, temp);
			if (!(acc_count % nb_acc_)) {
				acc_vec.resize(pitch_);
				data_.insert(data_.end(), acc_vec.begin(), acc_vec.end());
				acc_vec.assign(bb.buffer_size(), 0.0f);
			}
			acc_count++;
		}
		normalize(data_);
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
	uint32_t version = 2;
	fwrite(&version, sizeof(uint32_t), 1, fp);
	fwrite(&pitch_, sizeof(uint32_t), 1, fp);
	fwrite(&bunch_mask_, sizeof(uint32_t), 1, fp);
	fwrite(&nb_acc_, sizeof(uint32_t), 1, fp);
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
	fread(&bunch_mask_, sizeof(uint32_t), 1, fp);
	fread(&nb_acc_, sizeof(uint32_t), 1, fp);
	if (version == 2) { // load time (if v2)
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
	return data_.size() / pitch_;
}

const std::vector<float>& spectrogram::data() const {
	return data_;
}

const float* spectrogram::line(uint32_t index, uint32_t nb_lines) const {
	uint32_t total_lines = data_.size() / pitch_;
	if (nb_lines > total_lines)
		throw std::runtime_error("too many lines asked not that many available!");
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
