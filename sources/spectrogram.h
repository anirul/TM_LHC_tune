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

#ifndef spectrogram_HEADER_DEFINED
#define spectrogram_HEADER_DEFINED

#include <bitset>
#include <limits>

#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "bunch_buffer.h"

// functor to pass to load_file
struct commands {
	commands() {}
	virtual ~commands() {}
	virtual void operator()(
			bunch_buffer_f& bb,
			std::vector<float>& out) const = 0;
};

class spectrogram {
protected :
	uint32_t pitch_;
	uint32_t nb_acc_;
	std::bitset<16> bunch_mask_;
	std::vector<short> bunch_pattern_;
	std::vector<float> data_;
	std::vector<long long> time_;
protected :
	void accumulate(
			std::vector<float>& out,
			const std::vector<float>& in1,
			const std::vector<float>& in2) const;
	void divide(
			std::vector<float>& inout,
			float divider) const;
	boost::posix_time::time_duration normalize(
			std::vector<float>& inout) const;
	boost::posix_time::time_duration average(
			const bunch_buffer_f& buffers,
			std::vector<float>& out) const;
	bunch_buffer_f buffer_from_file(
			const boost::filesystem::path& path,
			i_fft_f* fft_instance) const;
	boost::posix_time::ptime ptime_from_file(
			const boost::filesystem::path& path) const;
	long long time_stamp_from_file(
			const boost::filesystem::path& path) const;
public :
	spectrogram(
			uint32_t nb_acc,
			const std::bitset<16>& bunch_mask);
	virtual ~spectrogram();
public :
	void load_files(
			const std::string& path, 
			const commands& cmd,
			i_fft_f* fft_instance,
			int64_t start_time = 0, 
			int64_t end_time = std::numeric_limits<int64_t>::max());
	void save_dump(const std::string& file) const;
	void load_dump(const std::string& file);
	void save_csv(const std::string& file, bool simple = true) const;
	uint32_t pitch() const;
	uint32_t line_count() const;
	const std::vector<float>& data() const;
	const std::vector<short>& bunch_pattern() const;
	const std::bitset<16>& bunch_mask() const;
	const float* line(uint32_t index, uint32_t nb_lines) const;
	long long time(uint32_t index) const;
};

#endif // spectrogram_HEADER_DEFINED
