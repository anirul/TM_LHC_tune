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

class spectrogram {
	protected :
		uint32_t pitch_;
		uint32_t bunch_mask_;
		uint32_t nb_acc_;
		std::vector<float> data_;
		std::vector<long long> time_;
	protected :
		void accumulate(
			std::vector<float>& out,
			const std::vector<float>& in1,
			const std::vector<float>& in2);
		void divide(
			std::vector<float>& inout,
			float divider);
		void normalize(std::vector<float>& inout);
		void average(
			const bunch_buffer_f& buffers,
			std::vector<float>& out,
			uint32_t bunch_mask);
	public :
		spectrogram(
			uint32_t nb_acc,
			uint32_t bunch_mask);
		virtual ~spectrogram();
	public :
		void load_files(const std::string& path, bool pre_notch = false);
		void save_dump(const std::string& file) const;
		void load_dump(const std::string& file);
		uint32_t pitch() const;
		uint32_t line_count() const;
		const std::vector<float>& data() const;
		const float* line(uint32_t index, uint32_t nb_lines) const;
		long long time(uint32_t index) const;
};

#endif // spectrogram_HEADER_DEFINED
