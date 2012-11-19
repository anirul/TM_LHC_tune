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

#ifndef bunch_buffer_HEADER_DEFINED
#define bunch_buffer_HEADER_DEFINED

#include <vector>
#include <limits>
#include <fstream>

#include "acquisition_buffer.h" 

class bunch_buffer_f {
   protected :
      std::vector<acquisition_buffer_f> buffers_;
      std::vector<short> bunch_pattern_;
   protected :
      void save_txt(
            std::ostream& os,
            const std::string& time_stamp);
      void load_txt(std::istream& is);
   public :
      // ctor data and the number of bunches
      bunch_buffer_f(
            const std::vector<short>& data, 
            const std::vector<short>& bunch_pattern);
      bunch_buffer_f(const std::string& file_name);
      virtual ~bunch_buffer_f();
   public :
      // peak detection of the buffers
      std::vector<unsigned long> peak_detect(
            const unsigned long min,
            const unsigned long max);
      std::vector<short> get_bunch_pattern() const;
      void buffer(
            const unsigned long index, 
            std::vector<float>& out) const;
      size_t bunch_count() const;
      size_t buffer_size() const;
      void notch();
      float check_rms();
      void svd();
      void fft();
      void amplitude();
      void phase_deg();
      void log10();
      bool save_txt(
            const std::string& file_name,
            const std::string& time_stamp);
      bool save_gzip(
            const std::string& file_name,
            const std::string& time_stamp);
      bool load_txt(const std::string& file_name);
      bool load_gzip(const std::string& file_name);
      bool empty() const;
};

class bunch_buffer_d {
   protected :
      std::vector<acquisition_buffer_d> buffers_;
      std::vector<short> bunch_pattern_;
   protected :
      void save_txt(
            std::ostream& os,
            const std::string& time_stamp);
      void load_txt(std::istream& is);
   public :
      // ctor data and the number of bunches
      bunch_buffer_d(
            const std::vector<short>& data, 
            const std::vector<short>& bunch_pattern);
      bunch_buffer_d(const std::string& file_name);
      virtual ~bunch_buffer_d();
   public :
      // peak detection of the buffers
      std::vector<unsigned long> peak_detect(
            const unsigned long min,
            const unsigned long max);
      std::vector<short> get_bunch_pattern() const;
      void buffer(
            const unsigned long index, 
            std::vector<double>& out) const;
      size_t bunch_count() const;
      size_t buffer_size() const;
      void notch();
      double check_rms();
      void svd();
      void fft();
      void amplitude();
      void phase_deg();
      void log10();
      bool save_txt(
            const std::string& file_name,
            const std::string& time_stamp);
      bool save_gzip(
            const std::string& file_name,
            const std::string& time_stamp);
      bool load_txt(const std::string& file_name);
      bool load_gzip(const std::string& file_name);
      bool empty() const;
};

#endif // bunch_buffer_HEADER_DEFINED

