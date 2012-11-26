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

#include "bunch_buffer.h"
#include "gsl_svd.h"


bunch_buffer_f::bunch_buffer_f() {}

bunch_buffer_d::bunch_buffer_d() {}

bunch_buffer_f::bunch_buffer_f(const bunch_buffer_f& bb) {
   buffers_.insert(
      buffers_.end(),
      bb.buffers_.begin(), 
      bb.buffers_.end());
   bunch_pattern_.insert(
      bunch_pattern_.end(),
      bb.bunch_pattern_.begin(), 
      bb.bunch_pattern_.end());
}

bunch_buffer_d::bunch_buffer_d(const bunch_buffer_d& bb) {
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
   const std::vector<short>& bunch_pattern)
{
   bunch_pattern_ = bunch_pattern;
   for (unsigned long i = 0; i < bunch_pattern_.size(); ++i) {
      buffers_.push_back(acquisition_buffer_f(data, bunch_pattern_.size(), i));
   }
}

bunch_buffer_d::bunch_buffer_d(
   const std::vector<short>& data,
   const std::vector<short>& bunch_pattern)
{
   bunch_pattern_ = bunch_pattern;
   for (unsigned long i = 0; i < bunch_pattern.size(); ++i) {
      buffers_.push_back(acquisition_buffer_d(data, bunch_pattern.size(), i));
   }
}

bunch_buffer_f::~bunch_buffer_f() {}

bunch_buffer_d::~bunch_buffer_d() {}

bunch_buffer_f::bunch_buffer_f(const std::string& file_name) {
   if (file_name.find(".gz") != std::string::npos) {
      load_gzip(file_name);
      return;
   }
   if (file_name.find(".xml") != std::string::npos) {
      load_txt(file_name);
      return;
   }
}

bunch_buffer_d::bunch_buffer_d(const std::string& file_name) {
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

void bunch_buffer_f::buffer(
   const unsigned long index,
   std::vector<float>& out) const
{
   if (index >= bunch_pattern_.size())
      throw std::runtime_error("bunch_buffer_f::buffer := index out of band");
   buffers_[index].buffer_real(out);
}

void bunch_buffer_d::buffer(
   const unsigned long index,
   std::vector<double>& out) const
{
   if (index >= bunch_pattern_.size())
      throw std::runtime_error("bunch_buffer_d::buffer := index out of band");
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

void bunch_buffer_f::notch() {
   for (unsigned long i = 0; i < bunch_pattern_.size(); ++i)
      buffers_[i].notch();
}

void bunch_buffer_d::notch() {
   for (unsigned long i = 0; i < bunch_pattern_.size(); ++i)
      buffers_[i].notch();
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

void bunch_buffer_f::svd() {
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
   SVD_mod(U, X, V, S, work);

   // sort S (by value)
   std::vector<double> s_sorted(bunch_count());
   for (size_t i = 0; i < bunch_count(); ++i)
      s_sorted[i] = S[i];
   std::sort(s_sorted.begin(), s_sorted.end());

   // S -> s
   gsl::matrix s(bunch_count(), bunch_count());
   for (size_t i = 0; i < bunch_count(); ++i) {
	   if (S[i] < (s_sorted[(size_t)((double)bunch_count() * 0.20)])) {
		   s(i, i) = 0.0f;
         continue;
      }
	   if (S[i] > (s_sorted[(size_t)((double)bunch_count() * 0.80)])) {
		   s(i, i) = 0.0f;
         continue;
      }
	   s(i, i) = S[i];
   }
   // vt = V^T
   gsl::matrix vt = transpose(V);
   // out = U s vt (out ~ A)
   gsl::matrix out = U * s * vt;
//   std::vector<double> deviation;
   for (int y = 0; y < bunch_count(); ++y) {
      for (int x = 0; x < bunch_count(); ++x) {
         buffers_[y][x] = out(y, x);
//         deviation.push_back(fabs(A(y, x) - out(y, x)));
      }
   }
//   std::cout << " svd deviation : " << average_d(deviation);
}

void bunch_buffer_d::svd() {
   throw std::runtime_error("not implemented");
}

void bunch_buffer_f::fft() {
   for (unsigned long i = 0; i < bunch_pattern_.size(); ++i)
      buffers_[i].fft();
}

void bunch_buffer_d::fft() {
   for (unsigned long i = 0; i < bunch_pattern_.size(); ++i)
      buffers_[i].fft();
}

void bunch_buffer_f::amplitude() {
   for (unsigned long i = 0; i < bunch_pattern_.size(); ++i)
      buffers_[i].amplitude();
}

void bunch_buffer_d::amplitude() {
   for (unsigned long i = 0; i < bunch_pattern_.size(); ++i)
      buffers_[i].amplitude();
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
         buffers_.push_back(acquisition_buffer_f(values));
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
         buffers_.push_back(acquisition_buffer_d(values));
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
