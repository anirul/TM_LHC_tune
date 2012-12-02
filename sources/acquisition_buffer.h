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

#ifndef acquisition_buffer_HEADER_DEFINED
#define acquisition_buffer_HEADER_DEFINED

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <vector>
#include <complex>
#include "fftw_fft.h"
#include "i_fft.h"

float rms_f(const std::vector<float>& vec);
float average_f(const std::vector<float>& vec);
float average_f(const std::vector<unsigned long>& vec);
double rms_d(const std::vector<double>& vec);
double average_d(const std::vector<double>& vec);
double average_d(const std::vector<unsigned long>& vec);

class acquisition_buffer_f {
protected :
	std::vector<std::complex<float> > complex_buffer_;
	i_fft_f* fft_instance_;
public :
	acquisition_buffer_f(
			const std::vector<short>& in,
			i_fft_f* fft_instance,
			unsigned long pitch,
			unsigned long offset);
	acquisition_buffer_f(
			const std::string& values,
			i_fft_f* fft_instance);
	void buffer_complex(std::vector<std::complex<float> >& out) const;
	void buffer_real(std::vector<float>& out) const;
	void save_txt(std::ostream& os);
	void load_txt(std::istream& is);
	size_t size() const;
      void clean(size_t begin, size_t end);
	void notch();
	void average();
	float check_rms();
	void fft();
	void amplitude();
	void phase_deg();
	void log10();
	unsigned long peak_detect(
			unsigned long min,
			unsigned long max);
	bool empty() const;
	void resize(size_t size);
	float& operator[](size_t index);
	const float& operator[](size_t index) const;
};

class acquisition_buffer_d {
protected :
	std::vector<std::complex<double> > complex_buffer_;
	i_fft_d* fft_instance_;
public :
	acquisition_buffer_d(
			const std::vector<short>& in,
			i_fft_d* fft_instance,
			unsigned long pitch,
			unsigned long offset);
	acquisition_buffer_d(
			const std::string& values,
			i_fft_d* fft_instance);
	void buffer_complex(std::vector<std::complex<double> >& out) const;
	void buffer_real(std::vector<double>& out) const;
	void save_txt(std::ostream& os);
	void load_txt(std::istream& is);
	size_t size() const;
      void clean(size_t begin, size_t end);
	void notch();
	void average();
	double check_rms();
	void fft();
	void amplitude();
	void phase_deg();
	void log10();
	unsigned long peak_detect(
			unsigned long min,
			unsigned long max);
	bool empty() const;
	void resize(size_t size);
	double& operator[](size_t index);
	const double& operator[](size_t index) const;
};

#endif // acquisition_buffer_HEADER_DEFINED

