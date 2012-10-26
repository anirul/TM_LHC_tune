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

void fftwf_fft::prepare(const std::vector<std::complex<float> >& in) {
   if (in.size() <= 0)
      throw std::runtime_error("unvalid buffer size");
   fftw_buffer_in_ = in;
   fftw_buffer_out_.resize(in.size());
}

void fftwd_fft::prepare(const std::vector<std::complex<double> >& in) {
   if (in.size() <= 0)
      throw std::runtime_error("unvalid buffer size");
   fftw_buffer_in_ = in;
   fftw_buffer_out_.resize(in.size());
}

boost::posix_time::time_duration fftwf_fft::run(
      std::vector<std::complex<float> >& out) 
{
   ptime before;
   ptime after;
   {
      boost::mutex::scoped_lock lock_it(lock_);
      fftwf_plan plan;
      before = microsec_clock::universal_time();
      plan = fftwf_plan_dft_1d(
            fftw_buffer_in_.size(),
            (fftwf_complex*)&fftw_buffer_in_[0],
            (fftwf_complex*)&fftw_buffer_out_[0],
            FFTW_FORWARD,
            FFTW_ESTIMATE);
      fftwf_execute(plan);
      after = microsec_clock::universal_time();
      fftwf_destroy_plan(plan);
   }
   out = fftw_buffer_out_;
   return after - before;
}

boost::posix_time::time_duration fftwd_fft::run(
      std::vector<std::complex<double> >& out)
{
   ptime before;
   ptime after;
   {
      boost::mutex::scoped_lock lock_it(lock_);
      fftw_plan plan;
      before = microsec_clock::universal_time();
      plan = fftw_plan_dft_1d(
            fftw_buffer_in_.size(),
            (fftw_complex*)&fftw_buffer_in_[0],
            (fftw_complex*)&fftw_buffer_out_[0],
            FFTW_FORWARD,
            FFTW_ESTIMATE);
      fftw_execute(plan);
      after = microsec_clock::universal_time();
      fftw_destroy_plan(plan);
   }
   out = fftw_buffer_out_;
   return after - before;
}

